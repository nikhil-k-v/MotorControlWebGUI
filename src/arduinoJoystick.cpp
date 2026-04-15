// Arduino UNO R3
// NRF24L01: REMOVED
// Joystick:  Vx=A2, Vy=A1 (analog), SW=D4 (button, pull-up)
// OLED SSD1306 128x64: SDA=A4, SCL=A5 (I2C)
//
// Communication: USB Serial at 115200 baud.
// Sends JoyPacket frames at up to 20 Hz, but ONLY when the
// position has changed by more than POS_CHANGE_THRESHOLD degrees
// or the button state has changed.  This keeps the byte rate low
// enough that the Web Serial API never overruns.
// Receives MotorPacket frames from the browser dashboard.
//
// Frame format (both directions):
//   [0x55] [0xAA] [LEN] [PAYLOAD of LEN bytes] [CRC8]
//   LEN = sizeof(JoyPacket) or sizeof(MotorPacket)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- OLED ----
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ---- Joystick ----
#define JOY_X_PIN  A2
#define JOY_Y_PIN  A1
#define JOY_SW_PIN 4

// ---- Serial framing ----
static const uint8_t FRAME_SYNC0 = 0x55;
static const uint8_t FRAME_SYNC1 = 0xAA;

// ---- Packet structs (must match Teensy exactly) ----
struct JoyPacket {
  float   pos_deg;
  uint8_t btn;      // 0x01 = FOC on, 0x02 = FOC off
};

struct MotorPacket {
  float   pos;
  float   vel;
  float   tau;
  uint8_t foc_on;
};

// ---- State ----
MotorPacket last_motor    = {0, 0, 0, 0};
uint32_t    last_motor_ms = 0;
bool        serial_ok     = false;   // true once we've received at least one good frame

bool foc_state        = false;
bool foc_state_latched = false;
bool last_btn_state   = HIGH;
uint32_t last_debounce_ms = 0;
static constexpr uint32_t DEBOUNCE_MS = 50;

float accumulated_pos_deg = 0.0f;
static constexpr float MAX_RATE_DEG_S = 60.0f;
static constexpr float POS_LIMIT_DEG  = 200.0f;

// TX rate limiting — send at most every TX_INTERVAL_MS,
// and only when something has actually changed.
static constexpr uint32_t TX_INTERVAL_MS       = 50;     // 20 Hz max
static constexpr float    POS_CHANGE_THRESHOLD = 0.15f;  // degrees
float    last_sent_pos_deg = -9999.0f;
uint8_t  last_sent_btn     = 0xFF;

// ============================================================
// CRC-8 (poly 0x07, init 0x00) — tiny and sufficient
// ============================================================
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// ============================================================
// Send a framed packet over Serial
// ============================================================
static void send_frame(const uint8_t *payload, uint8_t len) {
  Serial.write(FRAME_SYNC0);
  Serial.write(FRAME_SYNC1);
  Serial.write(len);
  Serial.write(payload, len);
  Serial.write(crc8(payload, len));
}

// ============================================================
// Receive state machine — call every loop
// Returns true and fills buf when a complete valid frame arrives
// ============================================================
static bool recv_frame(uint8_t *buf, uint8_t expected_len) {
  // State: 0=wait sync0, 1=wait sync1, 2=wait len, 3=payload, 4=crc
  static uint8_t  state    = 0;
  static uint8_t  rx_len   = 0;
  static uint8_t  rx_buf[64];
  static uint8_t  rx_idx   = 0;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    switch (state) {
      case 0:
        if (b == FRAME_SYNC0) state = 1;
        break;
      case 1:
        state = (b == FRAME_SYNC1) ? 2 : 0;
        break;
      case 2:
        rx_len = b;
        rx_idx = 0;
        // Guard against garbage lengths
        state = (rx_len > 0 && rx_len <= sizeof(rx_buf)) ? 3 : 0;
        break;
      case 3:
        rx_buf[rx_idx++] = b;
        if (rx_idx >= rx_len) state = 4;
        break;
      case 4: {
        state = 0;
        uint8_t expected_crc = crc8(rx_buf, rx_len);
        if (b == expected_crc && rx_len == expected_len) {
          memcpy(buf, rx_buf, rx_len);
          return true;
        }
        // CRC or length mismatch — discard
        break;
      }
    }
  }
  return false;
}

// ============================================================
// Joystick
// ============================================================
float read_joystick_rate() {
  int raw_y  = analogRead(JOY_X_PIN);
  int centred = raw_y - 512;
  if (centred > -60 && centred < 60) return 0.0f;
  float rate = (float)centred / 512.0f;
  return constrain(rate, -1.0f, 1.0f);
}

// ============================================================
// Button (debounced toggle)
// ============================================================
uint8_t read_button() {
  bool reading = digitalRead(JOY_SW_PIN);

  if (reading != last_btn_state) {
    last_debounce_ms = millis();
    last_btn_state   = reading;
  }

  if ((millis() - last_debounce_ms) > DEBOUNCE_MS && reading == LOW) {
    if (!foc_state_latched) {
      foc_state         = !foc_state;
      foc_state_latched = true;
    }
  } else if (reading == HIGH) {
    foc_state_latched = false;
  }

  return foc_state ? 0x01 : 0x02;
}

// ============================================================
// Joystick widget (unchanged from original)
// ============================================================
void draw_joystick_widget(int cx, int cy, int r, int16_t jx, int16_t jy) {
  display.drawCircle(cx, cy, r, WHITE);
  display.drawFastHLine(cx - r + 1, cy, (r * 2) - 1, WHITE);
  display.drawFastVLine(cx, cy - r + 1, (r * 2) - 1, WHITE);

  int dx = (int)(-(float)jy / 512.0f * (r - 4));
  int dy = (int)( (float)jx / 512.0f * (r - 4));

  int dot_x = constrain(cx + dx, cx - r + 2, cx + r - 2);
  int dot_y = constrain(cy + dy, cy - r + 2, cy + r - 2);

  display.fillCircle(dot_y, dot_x, 3, WHITE);
}

// ============================================================
// Display — updated to show "SER" instead of "RF"
// ============================================================
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

  // "SER" replaces "RF" — shows whether PC relay is alive
  display.setCursor(66, 22);
  display.print("SER:");
  display.print(serial_ok ? "OK" : "--");

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

// ============================================================
// setup
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // No serial print here — Serial is our comms link now
    // Just carry on; display won't work but comms will
  } else {
    display.setRotation(2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 28);
    display.print("Initialising...");
    display.display();
  }
}

// ============================================================
// loop
// ============================================================
void loop() {
  static uint32_t last_ms = millis();
  uint32_t now_ms         = millis();
  float    dt             = (now_ms - last_ms) / 1000.0f;
  last_ms                 = now_ms;

  // 1. Integrate joystick rate → position setpoint
  float rate = read_joystick_rate();
  accumulated_pos_deg += rate * MAX_RATE_DEG_S * dt;
  accumulated_pos_deg  = constrain(accumulated_pos_deg, -POS_LIMIT_DEG, POS_LIMIT_DEG);

  // 2. Build JoyPacket
  JoyPacket pkt;
  pkt.pos_deg = accumulated_pos_deg;
  pkt.btn     = read_button();

  // 3. Transmit only when something changed AND the interval has elapsed.
  //    This keeps the serial byte rate well within Web Serial's buffer limits.
  static uint32_t last_tx_ms = 0;
  bool pos_changed = fabsf(pkt.pos_deg - last_sent_pos_deg) > POS_CHANGE_THRESHOLD;
  bool btn_changed = (pkt.btn != last_sent_btn);
  if ((millis() - last_tx_ms >= TX_INTERVAL_MS) && (pos_changed || btn_changed)) {
    send_frame((const uint8_t *)&pkt, sizeof(pkt));
    last_tx_ms        = millis();
    last_sent_pos_deg = pkt.pos_deg;
    last_sent_btn     = pkt.btn;
  }

  // 4. Check for incoming MotorPacket frame from dashboard
  uint8_t rx_buf[sizeof(MotorPacket)];
  if (recv_frame(rx_buf, sizeof(MotorPacket))) {
    memcpy(&last_motor, rx_buf, sizeof(MotorPacket));
    last_motor_ms = millis();
    serial_ok     = true;
  }

  // 5. Update OLED ~30 Hz
  static uint32_t last_disp = 0;
  if (millis() - last_disp >= 33) {
    last_disp = millis();
    update_display(pkt);
  }

  // Small yield — loop runs fast, TX is now rate-limited above
  delay(5);
}