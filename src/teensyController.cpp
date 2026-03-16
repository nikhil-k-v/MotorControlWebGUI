// Teensy 4.1  ↔  MIT-style actuator over CAN (CAN3 pins 3/4)
// OLED: SSD1309 2.42" I2C (SDA=18, SCL=19)
// Web GUI serial protocol preserved:
//   Telemetry: T,<ms>,<id>,<mode>,<pos>,<vel>,<tau>,<rx_ok>,<rx_cnt>,<tx_cnt>
//   Menu/status: M,...

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

#define USE_IMU 0

#if USE_IMU
#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t bnoValue;
#endif

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CANbus;
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ============================================================
// ORIGINAL SPLASH LOGO BITMAP (reverted)
// ============================================================

static const uint8_t logo_bits[] PROGMEM = {
  0xe0,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0x03,0x00,0x00,
  0xf0,0xff,0xff,0xff,0xff,0x03,0x00,0x00,0xf0,0xff,0xff,0xff,0xff,0xc3,0x01,0x00,
  0xf0,0xff,0xff,0xff,0xff,0xe3,0x03,0x00,0xc0,0xff,0xff,0xff,0xff,0xf1,0x07,0x00,
  0x00,0x00,0x00,0x80,0xff,0xf1,0x07,0x00,0x00,0x00,0x00,0x00,0xff,0xf8,0x0f,0x00,
  0x00,0x00,0x00,0x00,0xff,0xf8,0x0f,0x00,0x00,0x00,0x00,0x00,0x7f,0xfc,0x1f,0x00,
  0x00,0x00,0x00,0x00,0x3f,0xfe,0x1f,0x00,0x00,0x00,0x00,0x80,0x3f,0xfe,0x3f,0x00,
  0x00,0x00,0x00,0xc0,0x1f,0xff,0x7f,0x00,0x00,0x00,0x00,0xc0,0x1f,0xff,0x7f,0x00,
  0x00,0x00,0x00,0xe0,0x8f,0x3f,0xfe,0x00,0x00,0x00,0x00,0xf0,0x8f,0x1f,0xfc,0x01,
  0x00,0x00,0x00,0xf0,0xc7,0x0f,0xfc,0x01,0x00,0x00,0x00,0xf0,0xe3,0x0f,0xf8,0x01,
  0x00,0x00,0x00,0xf8,0xe3,0x0f,0xf0,0x03,0x00,0x00,0x00,0xfc,0xf1,0x07,0xe0,0x07,
  0x00,0x00,0x00,0xfc,0xf0,0x03,0xe0,0x07,0xf0,0x00,0x00,0xfe,0xf8,0x01,0xe0,0x0f,
  0xf8,0x00,0x00,0xff,0xf8,0x01,0xc0,0x1f,0xf8,0x01,0x00,0x7f,0xfc,0x00,0xc0,0x1f,
  0xf8,0x03,0x00,0x3f,0xfe,0x00,0x80,0x1f,0xf8,0x03,0x80,0x3f,0x7e,0x00,0x00,0x1f,
  0xf0,0x07,0xc0,0x1f,0x7f,0x00,0x00,0x0e,0xf0,0x07,0xc0,0x0f,0x3f,0x00,0x00,0x00,
  0xe0,0x0f,0xe0,0x8f,0x3f,0x00,0x00,0x00,0xc0,0x1f,0xf0,0x8f,0x1f,0x00,0x00,0x00,
  0xc0,0x1f,0xf0,0xc7,0x0f,0x00,0x00,0x00,0x80,0x3f,0xf0,0xe3,0x0f,0x00,0x00,0x00,
  0x80,0x3f,0xf8,0xe3,0x07,0x00,0x00,0x00,0x00,0x7f,0xfc,0xf1,0x07,0x00,0x00,0x00,
  0x00,0xfe,0xff,0xf0,0x03,0x00,0x00,0x00,0x00,0xfe,0xff,0xf8,0x01,0x00,0x00,0x00,
  0x00,0xfc,0xff,0xf8,0x01,0x00,0x00,0x00,0x00,0xfc,0x7f,0xfc,0x00,0x00,0x00,0x00,
  0x00,0xf8,0x3f,0xfe,0x00,0x00,0x00,0x00,0x00,0xf8,0x3f,0xfe,0x00,0x00,0x00,0x00,
  0x00,0xf0,0x1f,0xff,0x00,0x00,0x00,0x00,0x00,0xf0,0x0f,0xff,0x00,0x00,0x00,0x00,
  0x00,0xe0,0x8f,0xff,0x03,0x00,0x00,0x00,0x00,0xc0,0xc7,0xff,0xff,0xff,0xff,0x0f,
  0x00,0x80,0xc7,0xff,0xff,0xff,0xff,0x0f,0x00,0x00,0xc0,0xff,0xff,0xff,0xff,0x0f,
  0x00,0x00,0xc0,0xff,0xff,0xff,0xff,0x0f,0x00,0x00,0x80,0xff,0xff,0xff,0xff,0x07
};

static constexpr int LOGO_W = 64;
static constexpr int LOGO_H = 48;

// ============================================================
// CENTRALIZED POSITION BOUNDS
// Change this one number to update:
//   1) Motor command clamp
//   2) OLED dial bounds
// ============================================================

static constexpr float POS_LIMIT_DEG = 200.0f;
static constexpr float POS_LIMIT_RAD = POS_LIMIT_DEG * PI / 180.0f;

// ============================================================
// UI layout
// ============================================================

static constexpr int SCREEN_W = 128;
static constexpr int SCREEN_H = 64;

static constexpr int TOP_ROW_H = 14;

static constexpr int WD_BOX_W = 40;
static constexpr int WD_BOX_H = 12;
static constexpr int WD_BOX_Y = 0;
static constexpr int WD_GAP   = 2;

// left column
static constexpr int SIDE_X = 0;
static constexpr int SIDE_Y = TOP_ROW_H + 1;
static constexpr int SIDE_W = 40;
static constexpr int FOC_H  = 12;
static constexpr int IMU_Y  = SIDE_Y + FOC_H + 2;
static constexpr int IMU_H  = SCREEN_H - IMU_Y;

// dial
static constexpr int DIAL_CX = 85;     // moved slightly left
static constexpr int DIAL_CY = 61;
static constexpr int DIAL_R_OUT = 35;  // wider
static constexpr int DIAL_R_IN  = 28;

// ============================================================
// Protocol limits
// ============================================================

static constexpr float P_MIN = -POS_LIMIT_RAD;
static constexpr float P_MAX =  POS_LIMIT_RAD;

static constexpr float V_MIN = -30.0f;
static constexpr float V_MAX =  30.0f;
static constexpr float KP_MIN=   0.0f;
static constexpr float KP_MAX= 500.0f;
static constexpr float KD_MIN=   0.0f;
static constexpr float KD_MAX= 100.0f;
static constexpr float T_MIN = -6.0f;
static constexpr float T_MAX =  6.0f;

static const uint8_t CMD_ENABLE [8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t CMD_DISABLE[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t CMD_ZERO   [8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

// ============================================================
// State
// ============================================================

enum CtrlMode { MODE_POS, MODE_VEL, MODE_TRQ };
volatile CtrlMode g_mode = MODE_POS;

volatile bool foc_on = false;
volatile uint8_t  motor_id = 1;
volatile uint16_t loop_hz  = 200;

volatile float cmd_p = 0.0f, cmd_v = 0.0f, cmd_t = 0.0f;
volatile float cmd_kp = 1.0f, cmd_kd = 1.0f;

volatile float motor_pos = 0.0f;
volatile float motor_vel = 0.0f;
volatile float motor_tau = 0.0f;

volatile uint32_t tx_count = 0, rx_count = 0;
volatile bool rx_ok = false;
uint32_t last_rx_ms = 0;

volatile bool scanning_active = false;
volatile bool suppress_tx = false;
volatile bool seen_id[128];

float imu_angle_deg = 0.0f;

// ============================================================
// IMU telemetry ring buffer
// ============================================================

static constexpr uint8_t IMU_LOG_LINES = 6;
static constexpr uint8_t IMU_LOG_LEN   = 18;
char imu_log[IMU_LOG_LINES][IMU_LOG_LEN];
uint8_t imu_log_head = 0;
uint8_t imu_log_count = 0;

// ============================================================
// Utils
// ============================================================

static inline float fclamp(float x, float a, float b){
  return x<a?a:(x>b?b:x);
}

static inline uint16_t float_to_u16(float x,float xmin,float xmax,int bits){
  x = fclamp(x,xmin,xmax);
  const float span = xmax-xmin;
  const float scale = (float)((1u<<bits)-1);
  return (uint16_t)((x-xmin)*(scale/span)+0.5f);
}

static inline float u16_to_float(uint16_t ui,float xmin,float xmax,int bits){
  const float scale = (float)((1u<<bits)-1);
  return ((float)ui)*(xmax-xmin)/scale+xmin;
}

static inline float rad_to_deg(float r){
  return r * 180.0f / PI;
}

static inline float mapf(float x, float in0, float in1, float out0, float out1){
  if (fabsf(in1 - in0) < 1e-6f) return out0;
  return out0 + (x - in0) * (out1 - out0) / (in1 - in0);
}

// ============================================================
// MIT-style CAN pack/unpack
// ============================================================

static void pack_cmd(uint8_t out[8], float p,float v,float kp,float kd,float t){
  const uint16_t P  = float_to_u16(p, P_MIN,  P_MAX, 16);
  const uint16_t V  = float_to_u16(v, V_MIN,  V_MAX, 12);
  const uint16_t KP = float_to_u16(kp,KP_MIN, KP_MAX,12);
  const uint16_t KD = float_to_u16(kd,KD_MIN, KD_MAX,12);
  const uint16_t T  = float_to_u16(t, T_MIN,  T_MAX, 12);

  out[0]=(P>>8)&0xFF; out[1]=(P>>0)&0xFF;
  out[2]=(V>>4)&0xFF; out[3]=((V&0x0F)<<4)|((KP>>8)&0x0F);
  out[4]=(KP>>0)&0xFF; out[5]=(KD>>4)&0xFF;
  out[6]=((KD&0x0F)<<4)|((T>>8)&0x0F); out[7]=(T>>0)&0xFF;
}

static void unpack_reply(const uint8_t in[8], float &p,float &v,float &t){
  const uint16_t P = ((uint16_t)in[0]<<8)|in[1];
  const uint16_t V = ((uint16_t)in[2]<<4)|(in[3]>>4);
  const uint16_t T = (((uint16_t)in[6]&0x0F)<<8)|in[7];
  p = u16_to_float(P,P_MIN,P_MAX,16);
  v = u16_to_float(V,V_MIN,V_MAX,12);
  t = u16_to_float(T,T_MIN,T_MAX,12);
}

// ============================================================
// CAN helpers
// ============================================================

static void can_send_bytes(uint8_t id,const uint8_t payload[8]){
  CAN_message_t m;
  m.id = id;
  m.len = 8;
  m.flags.extended = 0;
  for(int i=0;i<8;i++) m.buf[i]=payload[i];
  CANbus.write(m);
  tx_count++;
}

static void can_send_enable(bool on){
  foc_on = on;
  can_send_bytes(motor_id, on ? CMD_ENABLE : CMD_DISABLE);
  Serial.printf("M,FOC=%s\n", on ? "ON" : "OFF");
}

static void can_send_zero(){
  cmd_p = 0.0f;
  can_send_bytes(motor_id, CMD_ZERO);
  Serial.println("M,ZERO COMMAND SENT; PTARGET=0");
}

static void can_send_setpoints(){
  if(!foc_on || suppress_tx) return;

  uint8_t b[8];
  float p=0,v=0,kp=0,kd=0,t=0;

  switch(g_mode){
    case MODE_POS: p=cmd_p; kp=cmd_kp; kd=cmd_kd; break;
    case MODE_VEL: v=cmd_v; kd=cmd_kd; break;
    case MODE_TRQ: t=cmd_t; break;
  }

  pack_cmd(b,p,v,kp,kd,t);
  can_send_bytes(motor_id,b);
}

static inline uint8_t base_id_from_can(uint32_t raw){
  raw &= 0x7FF;
  if(raw>=1     && raw<=127)   return (uint8_t)raw;
  if(raw>=0x100 && raw<=0x17F) return (uint8_t)(raw-0x100);
  if(raw>=0x140 && raw<=0x1BF) return (uint8_t)(raw-0x140);
  if(raw>=0x200 && raw<=0x27F) return (uint8_t)(raw-0x200);
  if(raw>=0x240 && raw<=0x2BF) return (uint8_t)(raw-0x240);
  return 0;
}

// ============================================================
// RX ISR
// ============================================================

static void on_can_rx(const CAN_message_t &m){
  uint8_t bid = base_id_from_can(m.id);
  if(bid && bid < 128 && !seen_id[bid]){
    seen_id[bid] = true;
    if(scanning_active) Serial.printf("M,SCAN-FOUND,%u\n", bid);
  }

  if(m.len == 8){
    float p,v,t;
    unpack_reply(m.buf,p,v,t);

    motor_pos = p;
    motor_vel = v;
    motor_tau = t;

    rx_ok = true;
    last_rx_ms = millis();
    rx_count++;

    Serial.printf("T,%lu,%u,%s,%.6f,%.6f,%.6f,%d,%lu,%lu\n",
      millis(),
      (unsigned)motor_id,
      (g_mode==MODE_POS?"P":(g_mode==MODE_VEL?"V":"T")),
      p, v, t, 1,
      (unsigned long)rx_count,
      (unsigned long)tx_count
    );
  }
}

// ============================================================
// Tokenizer / parser
// ============================================================

struct Tok{ char* s; };

static bool eq_ci(const char* a,const char* b){
  while(*a && *b){
    char ca=(char)tolower((unsigned char)*a++);
    char cb=(char)tolower((unsigned char)*b++);
    if(ca!=cb) return false;
  }
  return *a==*b;
}

static int split_tokens(char* buf, Tok out[], int maxTok){
  int k=0;
  char* p=buf;
  while(*p && k<maxTok){
    while(*p && isspace((unsigned char)*p)) ++p;
    if(!*p) break;
    out[k].s = p;
    while(*p && !isspace((unsigned char)*p)) ++p;
    if(*p){ *p = '\0'; ++p; }
    ++k;
  }
  return k;
}

// ============================================================
// SCAN
// ============================================================

static void scan_motors(uint8_t s, uint8_t e){
  if(s<1) s=1;
  if(e>127) e=127;
  if(s>e){ uint8_t t=s; s=e; e=t; }

  Serial.printf("M,SCAN,STARTING,RANGE=%u-%u\n", s, e);

  for(int i=0;i<128;i++) seen_id[i]=false;

  scanning_active = true;
  suppress_tx = true;
  uint32_t tx0 = tx_count;

  uint8_t ping[8];
  pack_cmd(ping, 0,0,0,0,0);

  for(uint8_t id=s; id<=e; ++id){
    can_send_bytes(id, ping); delayMicroseconds(400);
    can_send_bytes(id, ping); delayMicroseconds(400);
  }

  const uint32_t until = millis() + 800;
  while((int32_t)(millis() - until) < 0){ yield(); }

  bool any=false;
  for(uint8_t id=s; id<=e; ++id){
    if(seen_id[id]){ any=true; break; }
  }

  if(!any && tx_count > tx0){
    seen_id[motor_id] = true;
    Serial.printf("M,SCAN-ASSUME,%u\n", motor_id);
  }

  Serial.print("M,SCAN-DONE,FOUND=");
  bool first=true;
  for(uint8_t id=s; id<=e; ++id){
    if(seen_id[id]){
      if(!first) Serial.print(' ');
      Serial.print(id);
      first=false;
    }
  }
  Serial.println();

  scanning_active = false;
  suppress_tx = false;
}

// ============================================================
// Menu / command handler
// ============================================================

static void menu_print_help(){
  Serial.println("M,COMMANDS: FOC 1|0 | MODE P|V|T | SET P <RAD>|V <RAD/S>|T <NM>|KP <..>|KD <..> | ID <N> | ZERO | RATE <HZ> | SCAN [START END]");
}

static void handle_line(char* buf){
  Tok t[8];
  int nt = split_tokens(buf,t,8);
  if(nt==0) return;

  auto toF=[&](int i){ return (i<nt)? atof(t[i].s):0.0f; };
  auto toI=[&](int i){ return (i<nt)? atoi(t[i].s):0; };

  if(eq_ci(t[0].s,"FOC")){
    bool on = (nt>=2 && toI(1)!=0);
    can_send_enable(on);
    return;
  }

  if(eq_ci(t[0].s,"MODE") && nt>=2){
    char c = t[1].s[0];
    if(c=='P'||c=='p'){ g_mode=MODE_POS; Serial.println("M,MODE=P"); }
    else if(c=='V'||c=='v'){ g_mode=MODE_VEL; Serial.println("M,MODE=V"); }
    else if(c=='T'||c=='t'){ g_mode=MODE_TRQ; Serial.println("M,MODE=T"); }
    return;
  }

  if(eq_ci(t[0].s,"SET") && nt>=3){
    if     (eq_ci(t[1].s,"P"))  cmd_p  = fclamp(toF(2), P_MIN, P_MAX);
    else if(eq_ci(t[1].s,"V"))  cmd_v  = fclamp(toF(2), V_MIN, V_MAX);
    else if(eq_ci(t[1].s,"T"))  cmd_t  = fclamp(toF(2), T_MIN, T_MAX);
    else if(eq_ci(t[1].s,"KP")) cmd_kp = fclamp(toF(2), 0.0f, 100.0f);
    else if(eq_ci(t[1].s,"KD")) cmd_kd = fclamp(toF(2), KD_MIN, KD_MAX);
    return;
  }

  if(eq_ci(t[0].s,"ZERO")){
    can_send_zero();
    return;
  }

  if(eq_ci(t[0].s,"ID") && nt>=2){
    motor_id = (uint8_t)constrain(toI(1),1,127);
    Serial.printf("M,ID=%u\n", (unsigned)motor_id);
    return;
  }

  if(eq_ci(t[0].s,"RATE") && nt>=2){
    loop_hz = (uint16_t)constrain(toI(1),10,1000);
    Serial.printf("M,RATE=%u HZ\n", loop_hz);
    return;
  }

  if(eq_ci(t[0].s,"SCAN")){
    uint8_t s=1,e=127;
    if(nt>=2) s=(uint8_t)constrain(toI(1),1,127);
    if(nt>=3) e=(uint8_t)constrain(toI(2),1,127);
    scan_motors(s,e);
    return;
  }

  if(eq_ci(t[0].s,"HELP")){
    menu_print_help();
    return;
  }

  Serial.println("M,? UNKNOWN. TRY HELP");
}

// ============================================================
// IMU helpers
// ============================================================

static void imu_log_append(const char* s){
  strncpy(imu_log[imu_log_head], s, IMU_LOG_LEN - 1);
  imu_log[imu_log_head][IMU_LOG_LEN - 1] = '\0';
  imu_log_head = (imu_log_head + 1) % IMU_LOG_LINES;
  if(imu_log_count < IMU_LOG_LINES) imu_log_count++;
}

static void imu_log_init(){
  imu_log_head = 0;
  imu_log_count = 0;
#if USE_IMU
  imu_log_append("IMU INIT");
#else
  imu_log_append("IMU OFF");
#endif
}

// ============================================================
// Splash
// ============================================================

static void showSplash(){
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawXBMP((SCREEN_W-LOGO_W)/2, (SCREEN_H-LOGO_H)/2, LOGO_W, LOGO_H, logo_bits);
  u8g2.sendBuffer();
  delay(5000);
}

// ============================================================
// UI helpers
// ============================================================

static uint8_t demo_seconds(){
  uint32_t s=(millis()/1000UL)%6UL;
  return (uint8_t)(5 - s);
}

static void drawWatchdog(int x,int y,int w,int h,const char*label,uint8_t sec){
  const bool expired = (sec == 0);

  char text[20];
  snprintf(text,sizeof(text),"%s %u",label,sec);

  if(!expired){
    u8g2.setDrawColor(1);
    u8g2.drawBox(x,y,w,h);
    u8g2.setDrawColor(0);
  }else{
    u8g2.setDrawColor(0);
    u8g2.drawBox(x,y,w,h);
    u8g2.setDrawColor(1);
  }

  u8g2.drawFrame(x,y,w,h);

  u8g2.setFont(u8g2_font_5x8_tr);

  int16_t textW = u8g2.getStrWidth(text);
  int16_t ascent = u8g2.getAscent();
  int16_t descent = u8g2.getDescent();
  int16_t textH = ascent - descent;

  int16_t tx = x + (w - textW) / 2;
  int16_t ty = y + (h - textH) / 2 + ascent;

  u8g2.drawStr(tx, ty, text);
  u8g2.setDrawColor(1);
}

static void drawFOCIndicator(){
  const int x = SIDE_X;
  const int y = SIDE_Y;
  const int w = SIDE_W;
  const int h = FOC_H;

  if(foc_on){
    u8g2.setDrawColor(1);
    u8g2.drawBox(x,y,w,h);
    u8g2.setDrawColor(0);
  }else{
    u8g2.setDrawColor(0);
    u8g2.drawBox(x,y,w,h);
    u8g2.setDrawColor(1);
  }

  u8g2.drawFrame(x,y,w,h);
  u8g2.setFont(u8g2_font_5x8_tr);

  const char* txt = foc_on ? "FOC ON" : "FOC OFF";
  int tw = u8g2.getStrWidth(txt);
  int ascent = u8g2.getAscent();
  int descent = u8g2.getDescent();
  int th = ascent - descent;

  int tx = x + (w - tw)/2;
  int ty = y + (h - th)/2 + ascent;

  u8g2.drawStr(tx, ty, txt);
  u8g2.setDrawColor(1);
}

static void drawIMUBox(){
  const int x = SIDE_X;
  const int y = IMU_Y;
  const int w = SIDE_W;
  const int h = IMU_H;

  u8g2.setDrawColor(1);
  u8g2.drawFrame(x,y,w,h);
  u8g2.setFont(u8g2_font_4x6_tr);

  int rows = (h - 4) / 6;
  if(rows < 1) rows = 1;
  if(rows > IMU_LOG_LINES) rows = IMU_LOG_LINES;

  int start = (int)imu_log_head - rows;
  while(start < 0) start += IMU_LOG_LINES;

  for(int i=0;i<rows;i++){
    int idx = (start + i) % IMU_LOG_LINES;
    u8g2.drawStr(x + 2, y + 6 + i*6, imu_log[idx]);
  }
}

static void drawArcSegment(int cx,int cy,int r,float deg0,float deg1){
  if(deg1 < deg0){
    float tmp = deg0;
    deg0 = deg1;
    deg1 = tmp;
  }

  const float step = 3.0f;
  float a = deg0;
  int px = cx + (int)roundf(cosf(a * PI / 180.0f) * r);
  int py = cy + (int)roundf(sinf(a * PI / 180.0f) * r);

  for(float d = deg0 + step; d <= deg1 + 0.1f; d += step){
    int x = cx + (int)roundf(cosf(d * PI / 180.0f) * r);
    int y = cy + (int)roundf(sinf(d * PI / 180.0f) * r);
    u8g2.drawLine(px, py, x, y);
    px = x;
    py = y;
  }
}

static void drawFilledArc(int cx, int cy, int rIn, int rOut, float deg0, float deg1){
  if(deg1 < deg0){
    float t = deg0;
    deg0 = deg1;
    deg1 = t;
  }

  const float step = 0.5f;   // smaller = smoother, denser fill

  for(float d = deg0; d <= deg1; d += step){
    float a = d * PI / 180.0f;

    int x1 = cx + (int)roundf(cosf(a) * rIn);
    int y1 = cy + (int)roundf(sinf(a) * rIn);
    int x2 = cx + (int)roundf(cosf(a) * rOut);
    int y2 = cy + (int)roundf(sinf(a) * rOut);

    u8g2.drawLine(x1, y1, x2, y2);
  }
}

static float display_pos_deg(){
  if(g_mode == MODE_POS) return rad_to_deg(cmd_p);
  return rad_to_deg(motor_pos);
}

static void drawDial(){
  const float pos_deg = fclamp(display_pos_deg(), -POS_LIMIT_DEG, POS_LIMIT_DEG);

  const float ARC_START = 190.0f;
  const float ARC_END   = 350.0f;
  const float MID_ANGLE = 0.5f * (ARC_START + ARC_END);
  const float valueDeg  = mapf(pos_deg, -POS_LIMIT_DEG, POS_LIMIT_DEG, ARC_START, ARC_END);

  // title
  u8g2.setFont(u8g2_font_4x6_tr);
  const char* title = "POSITION";
  int tw = u8g2.getStrWidth(title);
  u8g2.drawStr(DIAL_CX - tw/2, TOP_ROW_H + 7, title);

  // clean background arc: just two thin guide arcs, no filled dithery background
  drawArcSegment(DIAL_CX, DIAL_CY, DIAL_R_IN,  ARC_START, ARC_END);
  drawArcSegment(DIAL_CX, DIAL_CY, DIAL_R_OUT, ARC_START, ARC_END);

  // filled progress: always from left to current value, so 0 lands at midpoint
  drawFilledArc(DIAL_CX, DIAL_CY, DIAL_R_IN, DIAL_R_OUT, ARC_START, valueDeg);
  // center value
  char angleBuf[20];
  snprintf(angleBuf,sizeof(angleBuf),"%.1f",pos_deg);

  u8g2.setFont(u8g2_font_t0_12_tr);
  int vw = u8g2.getStrWidth(angleBuf);
  u8g2.drawStr(DIAL_CX - vw/2, DIAL_CY - 8, angleBuf);

  u8g2.setFont(u8g2_font_4x6_tr);
  const char* unit = "DEG";
  int uw = u8g2.getStrWidth(unit);
  u8g2.drawStr(DIAL_CX - uw/2, DIAL_CY + 2, unit);

  // endpoint labels tied to the same centralized bound
  char leftBuf[16], rightBuf[16];
  snprintf(leftBuf,  sizeof(leftBuf),  "-%.1f", POS_LIMIT_DEG);
  snprintf(rightBuf, sizeof(rightBuf),  "%.1f", POS_LIMIT_DEG);

  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(DIAL_CX - DIAL_R_OUT - 7, DIAL_CY + 3, leftBuf);
  u8g2.drawStr(DIAL_CX + DIAL_R_OUT - 13, DIAL_CY + 3, rightBuf);

  // midpoint tick for 0
  {
    float ar = MID_ANGLE * PI / 180.0f;
    int x1 = DIAL_CX + (int)roundf(cosf(ar) * (DIAL_R_IN - 2));
    int y1 = DIAL_CY + (int)roundf(sinf(ar) * (DIAL_R_IN - 2));
    int x2 = DIAL_CX + (int)roundf(cosf(ar) * (DIAL_R_OUT + 2));
    int y2 = DIAL_CY + (int)roundf(sinf(ar) * (DIAL_R_OUT + 2));
    u8g2.drawLine(x1,y1,x2,y2);
  }
}

static void renderDashboard(){
  uint8_t can_s = demo_seconds();
  uint8_t rf_s  = demo_seconds();
  uint8_t ctl_s = demo_seconds();

  u8g2.clearBuffer();

  drawWatchdog(0,                   WD_BOX_Y, WD_BOX_W, WD_BOX_H, "CAN",  can_s);
  drawWatchdog(WD_BOX_W + WD_GAP,   WD_BOX_Y, WD_BOX_W, WD_BOX_H, "RF",   rf_s);
  drawWatchdog(2*(WD_BOX_W + WD_GAP), WD_BOX_Y, WD_BOX_W, WD_BOX_H, "CTRL", ctl_s);

  drawFOCIndicator();
  drawIMUBox();
  drawDial();

  u8g2.sendBuffer();
}

// ============================================================
// IMU update
// ============================================================

static void updateIMU(){
#if USE_IMU
  static uint32_t last_log_ms = 0;

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    imu_log_append("IMU RESET");
  }

  if (bno08x.getSensorEvent(&bnoValue)) {
    if (bnoValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = bnoValue.un.rotationVector.real;
      float qx = bnoValue.un.rotationVector.i;
      float qy = bnoValue.un.rotationVector.j;
      float qz = bnoValue.un.rotationVector.k;

      float siny_cosp = 2.0f * (qw * qz + qx * qy);
      float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      imu_angle_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;

      if (millis() - last_log_ms >= 200) {
        last_log_ms = millis();
        char line[IMU_LOG_LEN];
        snprintf(line, sizeof(line), "%.1f DEG", imu_angle_deg);
        imu_log_append(line);
      }
    }
  }
#else
  // IMU disabled
#endif
}

// ============================================================
// setup / loop
// ============================================================

void setup(){
  Serial.begin(230400);
  while(!Serial && millis()<2000){}

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();

  u8g2.begin();

  imu_log_init();
  showSplash();

#if USE_IMU
  if (bno08x.begin_I2C(0x4A, &Wire) || bno08x.begin_I2C(0x4B, &Wire)) {
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    imu_log_append("IMU OK");
  } else {
    imu_log_append("IMU FAIL");
  }
#endif

  CANbus.begin();
  CANbus.setBaudRate(1000000);
  CANbus.setMaxMB(16);
  CANbus.enableFIFO();
  CANbus.enableFIFOInterrupt();
  CANbus.onReceive(on_can_rx);
  CANbus.mailboxStatus();

  Serial.println("M,TEENSY READY");
  menu_print_help();
  Serial.println("M,CAN @ 1M READY");
}

void loop(){
  while(Serial.available()){
    static char sbuf[128];
    static size_t slen=0;
    int c = Serial.read();

    if(c=='\r') continue;

    if(c=='\n'){
      sbuf[slen]='\0';
      handle_line(sbuf);
      slen=0;
    }else if(slen < sizeof(sbuf)-1){
      sbuf[slen++] = (char)c;
    }
  }

  static uint32_t last_us = 0;
  const uint32_t period_us = 1000000UL / (uint32_t)loop_hz;
  const uint32_t now_us = micros();

  if(now_us - last_us >= period_us){
    last_us = now_us;
    if(!scanning_active) can_send_setpoints();
    if(millis() - last_rx_ms > 200) rx_ok = false;
  }

  updateIMU();

  static uint32_t last_display_ms = 0;
  const uint32_t now_ms = millis();
  if(now_ms - last_display_ms >= 33){
    last_display_ms = now_ms;
    renderDashboard();
  }
}