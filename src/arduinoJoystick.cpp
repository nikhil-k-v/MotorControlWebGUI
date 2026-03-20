// Arduino UNO R3
// NRF24L01: CE=D8, CSN=D10, SCK=D13, MOSI=D11, MISO=D12, VCC=3.3V, GND=GND
// Joystick:  Vx=A2, Vy=A1 (analog), SW=D2 (button, pull-up)
// OLED SSD1306 128x64: SDA=A4, SCL=A5 (I2C)

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- NRF24 ----
#define RF_CE  8
#define RF_CSN 10

RF24 radio(RF_CE, RF_CSN);

const uint64_t PIPE_TX = 0xF0F0F0F0D2LL;
const uint64_t PIPE_RX = 0xF0F0F0F0E1LL;

struct JoyPacket {
  float   pos_deg; // accumulated position setpoint in degrees
  uint8_t btn;     // bit1 = FOC off
};

struct MotorPacket {
  float pos;
  float vel;
  float tau;
  uint8_t foc_on;
};

// ---- Joystick ----
#define JOY_X_PIN  A2
#define JOY_Y_PIN  A1
#define JOY_SW_PIN 4

// ---- OLED ----
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ---- State ----
MotorPacket last_motor    = {0, 0, 0, 0};
uint32_t    last_motor_ms = 0;
bool        rf_ok         = false;
bool foc_state_latched = false;

// Accumulated position setpoint — starts at 0, joystick nudges it
float accumulated_pos_deg = 0.0f;

// Max rate of change in degrees per second at full joystick deflection
static constexpr float MAX_RATE_DEG_S = 60.0f;

// Position limits matching Teensy POS_LIMIT_DEG
static constexpr float POS_LIMIT_DEG = 200.0f;

// ---- Joystick read ----
// Returns a rate in -1..1 based on Y deflection, with dead zone
float read_joystick_rate() {
  int raw_y   = analogRead(JOY_X_PIN);
  int centred = raw_y - 512; // -512..511

  // dead zone: clamp -60..60 to 0
  if (centred > -60 && centred < 60) return 0.0f;

  // normalise to -1..1
  float rate = (float)centred / 512.0f;
  rate = constrain(rate, -1.0f, 1.0f);
  return rate;
}

// ---- Button state ----
bool     foc_state        = false;  // toggled on each click
bool     last_btn_state   = HIGH;
uint32_t last_debounce_ms = 0;
static constexpr uint32_t DEBOUNCE_MS = 50;

// Call this every loop — returns updated btn byte
uint8_t read_button() {
  bool reading = digitalRead(JOY_SW_PIN);

  if (reading != last_btn_state) {
    last_debounce_ms = millis();
    last_btn_state = reading;  // update ONLY on change
  }

  if ((millis() - last_debounce_ms) > DEBOUNCE_MS && reading == LOW) {
    if (!foc_state_latched) {
      foc_state = !foc_state;
      foc_state_latched = true;
      Serial.print("FOC toggled: ");
      Serial.println(foc_state ? "ON" : "OFF");
    }
  } else if (reading == HIGH) {
    foc_state_latched = false;
  }

  return foc_state ? 0x01 : 0x02;
}

JoyPacket build_packet(float pos_deg) {
  JoyPacket p;
  p.pos_deg = pos_deg;
  p.btn     = read_button();
  return p;
}

// ---- Joystick widget ----
void draw_joystick_widget(int cx, int cy, int r, int16_t jx, int16_t jy) {
  display.drawCircle(cx, cy, r, WHITE);
  display.drawFastHLine(cx - r + 1, cy, (r * 2) - 1, WHITE);
  display.drawFastVLine(cx, cy - r + 1, (r * 2) - 1, WHITE);

  int dx = (int)((float)jy / 512.0f * (r - 4));
  int dy = (int)(-(float)jx / 512.0f * (r - 4));

  int dot_x = constrain(cx + dx, cx - r + 2, cx + r - 2);
  int dot_y = constrain(cy + dy, cy - r + 2, cy + r - 2);

  display.fillCircle(dot_x, dot_y, 3, WHITE);
}

// ---- Display update ----
void update_display(JoyPacket &pkt) {
  int16_t raw_x = (int16_t)(analogRead(JOY_X_PIN) - 512);
  int16_t raw_y = (int16_t)(analogRead(JOY_Y_PIN) - 512);

  display.clearDisplay();

  draw_joystick_widget(30, 32, 28, raw_x, raw_y);

  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(66, 0);
  display.print("SP:");
  display.print(pkt.pos_deg, 1);

  display.setCursor(66, 10);
  display.print("FOC:");
  display.print(foc_state ? "ON" : "OF");

  display.setCursor(66, 22);
  display.print("RF:");
  display.print(rf_ok ? "OK" : "--");

  bool motor_fresh = (millis() - last_motor_ms < 500);

  display.setCursor(66, 34);
  display.print("MP:");
  if (motor_fresh) display.print(last_motor.pos, 1);
  else             display.print("---");

  display.setCursor(66, 44);
  display.print("MV:");
  if (motor_fresh) display.print(last_motor.vel, 1);
  else             display.print("---");

  display.setCursor(66, 54);
  display.print("F:");
  if (motor_fresh) display.print(last_motor.foc_on ? "ON" : "OF");
  else             display.print("--");

  display.display();
}

void setup() {
  Serial.begin(115200);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED INIT FAIL");
  } else {
    display.setRotation(2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 28);
    display.print("Initialising...");
    display.display();
  }

  if (!radio.begin()) {
    Serial.println("RF24 INIT FAIL");
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setRetries(3, 5);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  radio.openWritingPipe(PIPE_TX);
  radio.openReadingPipe(1, PIPE_RX);
  radio.stopListening();

  Serial.println("READY");
}

void loop() {
  static uint32_t last_ms = millis();
  uint32_t now_ms         = millis();
  float    dt             = (now_ms - last_ms) / 1000.0f;
  last_ms                 = now_ms;

  // 1. Read joystick rate and integrate into position setpoint
  float rate = read_joystick_rate();
  accumulated_pos_deg += rate * MAX_RATE_DEG_S * dt;
  accumulated_pos_deg  = constrain(accumulated_pos_deg, -POS_LIMIT_DEG, POS_LIMIT_DEG);

  // 2. Build and transmit packet
  JoyPacket pkt = build_packet(accumulated_pos_deg);
  rf_ok = radio.write(&pkt, sizeof(pkt));

  // 3. Ack payload = motor state from Teensy
  if (rf_ok && radio.isAckPayloadAvailable()) {
    radio.read(&last_motor, sizeof(last_motor));
    last_motor_ms = millis();
  }

  // 4. Update OLED ~30 Hz
  static uint32_t last_disp = 0;
  if (millis() - last_disp >= 33) {
    last_disp = millis();
    update_display(pkt);
  }

  // 5. Debug serial at 5 Hz
  static uint32_t last_print = 0;
  if (millis() - last_print >= 200) {
    last_print = millis();
    Serial.print("sp=");  Serial.print(pkt.pos_deg, 1);
    Serial.print(" btn="); Serial.print(pkt.btn);
    Serial.print(" rf=");  Serial.println(rf_ok ? "OK" : "FAIL");
  }

  // 6. 50 Hz TX rate
  delay(20);
}