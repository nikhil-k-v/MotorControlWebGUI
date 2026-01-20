// Teensy 4.1  ↔  MIT-style actuator over CAN (CAN2 pins 3/4)
// Serial protocol: Telemetry lines start with "T,", Menu/status with "M,"
// Requires: FlexCAN_T4 (Teensy package). USB Serial: 230400 | CAN: 1,000,000

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <ctype.h>
#include <stdlib.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CANbus;

// ==== protocol limits (MIT/YOBOTICS style) ====
static constexpr float P_MIN = -4.0f * PI, P_MAX =  4.0f * PI;
static constexpr float V_MIN = -30.0f,     V_MAX = 30.0f;
static constexpr float KP_MIN= 0.0f,       KP_MAX= 500.0f;
static constexpr float KD_MIN= 0.0f,       KD_MAX= 100.0f;
static constexpr float T_MIN = -18.0f,     T_MAX = 18.0f;

static const uint8_t CMD_ENABLE [8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_DISABLE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_ZERO   [8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

enum CtrlMode { MODE_POS, MODE_VEL, MODE_TRQ };
volatile CtrlMode g_mode = MODE_POS;

volatile bool foc_on = false;
volatile uint8_t  motor_id = 1;
volatile uint16_t loop_hz  = 200;

volatile float cmd_p = 0.0f, cmd_v = 0.0f, cmd_t = 0.0f;
volatile float cmd_kp = 10.0f, cmd_kd = 20.0f;

volatile uint32_t tx_count=0, rx_count=0;
volatile bool rx_ok=false;
uint32_t last_rx_ms=0;

// scan state
volatile bool scanning_active=false;
volatile bool suppress_tx=false;
volatile bool seen_id[128];  // index by CAN ID (1..127)

// ==== utils ====
static inline float fclamp(float x, float a, float b){ return x<a?a:(x>b?b:x); }
static inline uint16_t float_to_u16(float x,float xmin,float xmax,int bits){
  x=fclamp(x,xmin,xmax); const float span=xmax-xmin, scale=(float)((1u<<bits)-1);
  return (uint16_t)((x-xmin)*(scale/span)+0.5f);
}
static inline float u16_to_float(uint16_t ui,float xmin,float xmax,int bits){
  const float scale=(float)((1u<<bits)-1); return ((float)ui)*(xmax-xmin)/scale+xmin;
}

// ==== pack/unpack (MIT style) ====
static void pack_cmd(uint8_t out[8], float p,float v,float kp,float kd,float t){
  const uint16_t P=float_to_u16(p,P_MIN,P_MAX,16);
  const uint16_t V=float_to_u16(v,V_MIN,V_MAX,12);
  const uint16_t KP=float_to_u16(kp,KP_MIN,KP_MAX,12);
  const uint16_t KD=float_to_u16(kd,KD_MIN,KD_MAX,12);
  const uint16_t T=float_to_u16(t,T_MIN,T_MAX,12);
  out[0]=(P>>8)&0xFF; out[1]=(P>>0)&0xFF;
  out[2]=(V>>4)&0xFF; out[3]=((V&0x0F)<<4)|((KP>>8)&0x0F);
  out[4]=(KP>>0)&0xFF; out[5]=(KD>>4)&0xFF;
  out[6]=((KD&0x0F)<<4)|((T>>8)&0x0F); out[7]=(T>>0)&0xFF;
}
static void unpack_reply(const uint8_t in[8], float &p,float &v,float &t){
  const uint16_t P=((uint16_t)in[0]<<8)|in[1];
  const uint16_t V=((uint16_t)in[2]<<4)|(in[3]>>4);
  const uint16_t T=(((uint16_t)in[6]&0x0F)<<8)|in[7];
  p=u16_to_float(P,P_MIN,P_MAX,16);
  v=u16_to_float(V,V_MIN,V_MAX,12);
  t=u16_to_float(T,T_MIN,T_MAX,12);
}

// ==== CAN helpers ====
static void can_send_bytes(uint8_t id,const uint8_t payload[8]){
  CAN_message_t m; m.id=id; m.len=8; m.flags.extended=0;
  for(int i=0;i<8;i++) m.buf[i]=payload[i];
  CANbus.write(m); tx_count++;
}
static void can_send_enable(bool on){
  can_send_bytes(motor_id, on?CMD_ENABLE:CMD_DISABLE);
  Serial.printf("M,FOC=%s\n", on?"ON":"OFF");
}
static void can_send_zero(){
  cmd_p = 0.0f;              // prevent snap-back
  can_send_bytes(motor_id, CMD_ZERO);
  Serial.println("M,Zero command sent; Ptarget=0");
}

static void can_send_setpoints(){
  if(!foc_on || suppress_tx) return;
  uint8_t b[8]; float p=0,v=0,kp=0,kd=0,t=0;
  switch(g_mode){
    case MODE_POS: p=cmd_p; kp=cmd_kp; kd=cmd_kd; break;
    case MODE_VEL: v=cmd_v; kd=cmd_kd; break;
    case MODE_TRQ: t=cmd_t; break;
  }
  pack_cmd(b,p,v,kp,kd,t); can_send_bytes(motor_id,b);
}

// Decode “base” motor ID from an incoming CAN id (handles +0x100 replies)
static inline uint8_t base_id_from_can(uint32_t raw){
  raw &= 0x7FF;
  if(raw>=1    && raw<=127)  return (uint8_t)raw;       // same ID
  if(raw>=0x100&& raw<=0x17F) return (uint8_t)(raw-0x100); // MIT reply
  if(raw>=0x140&& raw<=0x1BF) return (uint8_t)(raw-0x140); // common vendor offset
  if(raw>=0x200&& raw<=0x27F) return (uint8_t)(raw-0x200);
  if(raw>=0x240&& raw<=0x2BF) return (uint8_t)(raw-0x240);
  return 0;
}

// ==== RX ISR ====
static void on_can_rx(const CAN_message_t &m){
  // mark discovery if scanning
  uint8_t bid=base_id_from_can(m.id);
  if(bid && bid<128 && !seen_id[bid]){
    seen_id[bid]=true;
    if(scanning_active) Serial.printf("M,SCAN-FOUND,%u\n", bid);
  }
  if(m.len==8){
    float p,v,t; unpack_reply(m.buf,p,v,t);
    rx_ok=true; last_rx_ms=millis(); rx_count++;
    Serial.printf("T,%lu,%u,%s,%.6f,%.6f,%.6f,%d,%lu,%lu\n",
      millis(), (unsigned)motor_id,
      (g_mode==MODE_POS?"P":(g_mode==MODE_VEL?"V":"T")),
      p,v,t,1,(unsigned long)rx_count,(unsigned long)tx_count);
  }
}

// ==== tiny tokenizer / ci-compare ====
struct Tok{ char* s; };
static bool eq_ci(const char* a,const char* b){
  while(*a && *b){ char ca=(char)tolower((unsigned char)*a++), cb=(char)tolower((unsigned char)*b++); if(ca!=cb) return false; }
  return *a==*b;
}
static int split_tokens(char* buf, Tok out[], int maxTok){
  int k=0; char* p=buf;
  while(*p && k<maxTok){
    while(*p && isspace((unsigned char)*p)) ++p; if(!*p) break;
    out[k].s=p; while(*p && !isspace((unsigned char)*p)) ++p; if(*p){ *p='\0'; ++p; }
    ++k;
  }
  return k;
}

// ==== SCAN ====
static void scan_motors(uint8_t s, uint8_t e){
  if(s<1) s=1; if(e>127) e=127; if(s>e){ uint8_t t=s; s=e; e=t; }
  Serial.printf("M,SCAN,starting,range=%u-%u\n", s, e);
  for(int i=0;i<128;i++) seen_id[i]=false;

  scanning_active=true;  // pause TX while scanning
  suppress_tx=true;
  uint32_t tx0 = tx_count;

  uint8_t ping[8]; pack_cmd(ping, 0,0,0,0,0);
  for(uint8_t id=s; id<=e; ++id){
    can_send_bytes(id, ping); delayMicroseconds(400);
    can_send_bytes(id, ping); delayMicroseconds(400);
  }

  // Passive listen window (longer)
  const uint32_t until = millis() + 800;
  while((int32_t)(millis() - until) < 0){ yield(); }

  // Fallback: if nothing heard but we have been transmitting, assume current ID is present
  bool any=false; for(uint8_t id=s; id<=e; ++id) if(seen_id[id]){ any=true; break; }
  if(!any && tx_count > tx0){
    seen_id[motor_id] = true;
    Serial.printf("M,SCAN-ASSUME,%u\n", motor_id);
  }

  Serial.print("M,SCAN-DONE,found=");
  bool first=true;
  for(uint8_t id=s; id<=e; ++id){
    if(seen_id[id]){ if(!first) Serial.print(' '); Serial.print(id); first=false; }
  }
  Serial.println();

  scanning_active=false; suppress_tx=false;
}


// ==== help + command handler ====
static void menu_print_help(){
  Serial.println("M,Commands: FOC 1|0 | MODE P|V|T | SET P <rad>|V <rad/s>|T <Nm>|KP <..>|KD <..> | ID <n> | ZERO | RATE <Hz> | SCAN [start end]");
}
static void handle_line(char* buf){
  Tok t[8]; int nt=split_tokens(buf,t,8); if(nt==0) return;
  auto toF=[&](int i){ return (i<nt)? atof(t[i].s):0.0; };
  auto toI=[&](int i){ return (i<nt)? atoi(t[i].s):0;   };

  if(eq_ci(t[0].s,"FOC")){ bool on=(nt>=2 && toI(1)!=0); foc_on=on; can_send_enable(on); return; }
  if(eq_ci(t[0].s,"MODE") && nt>=2){
    char c=t[1].s[0];
    if(c=='P'||c=='p'){ g_mode=MODE_POS; Serial.println("M,Mode=P"); }
    else if(c=='V'||c=='v'){ g_mode=MODE_VEL; Serial.println("M,Mode=V"); }
    else if(c=='T'||c=='t'){ g_mode=MODE_TRQ; Serial.println("M,Mode=T"); }
    return;
  }
  // Change the SET branch in handle_line():
  if(eq_ci(t[0].s,"SET") && nt>=3){
    if     (eq_ci(t[1].s,"P"))  cmd_p  = fclamp(toF(2), P_MIN, P_MAX);
    else if(eq_ci(t[1].s,"V"))  cmd_v  = fclamp(toF(2), V_MIN, V_MAX);
    else if(eq_ci(t[1].s,"T"))  cmd_t  = fclamp(toF(2), T_MIN, T_MAX);
    else if(eq_ci(t[1].s,"KP")) cmd_kp = fclamp(toF(2), 0.0f, 100.0f); // KP now 0..100
    else if(eq_ci(t[1].s,"KD")) cmd_kd = fclamp(toF(2), KD_MIN, KD_MAX);
    Serial.printf("M,Set OK KP=%.3f KD=%.3f\n", cmd_kp, cmd_kd);
    return;
  }

  if(eq_ci(t[0].s,"ZERO")){ can_send_zero(); return; }
  if(eq_ci(t[0].s,"ID") && nt>=2){ motor_id=(uint8_t)constrain(toI(1),1,127); Serial.printf("M,ID=%u\n",(unsigned)motor_id); return; }
  if(eq_ci(t[0].s,"RATE") && nt>=2){ loop_hz=(uint16_t)constrain(toI(1),10,1000); Serial.printf("M,Rate=%u Hz\n",loop_hz); return; }
  if(eq_ci(t[0].s,"SCAN")){
    uint8_t s=1,e=127; if(nt>=2) s=(uint8_t)constrain(toI(1),1,127); if(nt>=3) e=(uint8_t)constrain(toI(2),1,127);
    scan_motors(s,e); return;
  }
  if(eq_ci(t[0].s,"HELP")){ menu_print_help(); return; }
  Serial.println("M,? Unknown. Try HELP");
}

// ==== setup/loop ====
void setup(){
  Serial.begin(230400);
  while(!Serial && millis()<2000){}
  Serial.println("M,Teensy ready"); menu_print_help();

  CANbus.begin(); CANbus.setBaudRate(1000000);
  CANbus.setMaxMB(16); CANbus.enableFIFO(); CANbus.enableFIFOInterrupt();
  CANbus.onReceive(on_can_rx); CANbus.mailboxStatus();

  Serial.println("M,SCAN tip: type 'SCAN' to discover CAN IDs");
  Serial.println("M,CAN @ 1M ready");
}
void loop(){
  // serial intake
  while(Serial.available()){
    static char sbuf[128]; static size_t slen=0; int c=Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){ sbuf[slen]='\0'; handle_line(sbuf); slen=0; }
    else if(slen<sizeof(sbuf)-1){ sbuf[slen++]=(char)c; }
  }

  // periodic command streaming + heartbeat
  static uint32_t last_us=0; const uint32_t period_us = 1000000UL/ (uint32_t)loop_hz; const uint32_t now=micros();
  if(now - last_us >= period_us){
    last_us=now;
    if(!scanning_active) can_send_setpoints();
    if(millis()-last_rx_ms > 200) rx_ok=false;
    Serial.printf("T,%lu,%u,%s,%.6f,%.6f,%.6f,%d,%lu,%lu\n",
      millis(), (unsigned)motor_id,
      (g_mode==MODE_POS?"P":(g_mode==MODE_VEL?"V":"T")),
      cmd_p, cmd_v, cmd_t,          // <-- show commanded values
      rx_ok?1:0, (unsigned long)rx_count, (unsigned long)tx_count);
  }
}
