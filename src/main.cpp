#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include "driver/ledc.h"

// ---------------- Pin definitions ----------------
#define MOTOR_IN1   26
#define MOTOR_IN2   27
#define ENCODER_A   34
#define ENCODER_B   35

#define SERVO1_PIN  21
#define SERVO2_PIN  18

// Proximity sensor (ACTIVE LOW at home)
#define SENSOR_PIN  14
#define SENSOR_ACTIVE()    (digitalRead(SENSOR_PIN) == LOW)
#define SENSOR_INACTIVE()  (!SENSOR_ACTIVE())

// ---------------- Wi-Fi settings -----------------
// Preferred STA network
#define STA_SSID  "ROBOT"
#define STA_PASS  "Robotic@123"
// Desired final static IP: keep gateway subnet, force last octet to 51
#define DESIRED_LAST_OCTET  51

// Fallback AP (if STA fails)
#define AP_SSID   "AI_Trash_Bin"
#define AP_PASS   "sgt@since2022"

// --------------- Pulses & bins -------------------
#define PULSES_PER_REV   23550L            // thực tế bạn đo
#define PULSES_PER_BIN   (PULSES_PER_REV/4)

// --------------- Motor speeds (0..255) ----------
#define DUTY_MANUAL       128              // 50% cho Turn Left/Right (nút thủ công)
#define DUTY_FAST         120              // chạy nhanh khi còn xa
#define DUTY_MEDIUM       100              // trung bình khi gần
#define DUTY_SLOW          90              // chậm khi vào đích
#define DUTY_HOME_SLOW    100              // reset: quay chậm 1 chiều

// --------------- Decel thresholds ---------------
#define THRESH_MED       1600              // <= ngưỡng -> medium
#define THRESH_SLOW       500              // <= ngưỡng -> slow

// --------------- Web server ---------------------
WebServer server(80);

// --------------- Encoder state ------------------
volatile long encoderCount = 0;
volatile int  lastEncoded  = 0;

// --------------- Bin state ----------------------
int currentBin  = 1; // 1..4
int previousBin = 1;

// ---------- Helper cập nhật trạng thái ----------
inline void setCurrentBin(int newBin, bool updatePrevious = true) {
  if (updatePrevious) previousBin = currentBin;
  currentBin = newBin;
}

// ---------------- Servo driver ------------------
class ServoLite {
public:
  bool attach(int pin, int minUs = 500, int maxUs = 2500,
              ledc_timer_t timer = LEDC_TIMER_1, ledc_channel_t channel = LEDC_CHANNEL_2) {
    _pin = pin; _minUs = minUs; _maxUs = maxUs; _timer = timer; _channel = channel;

    // 50Hz, 16-bit
    ledc_timer_config_t cfg = {};
    cfg.speed_mode      = LEDC_HIGH_SPEED_MODE;
    cfg.duty_resolution = LEDC_TIMER_16_BIT;
    cfg.timer_num       = _timer;
    cfg.freq_hz         = 50;
    cfg.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&cfg);

    ledc_channel_config_t ch = {};
    ch.gpio_num   = _pin;
    ch.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch.channel    = _channel;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ch.timer_sel  = _timer;
    ch.duty       = 0;
    ch.hpoint     = 0;
    ledc_channel_config(&ch);

    write(0);
    return true;
  }

  void write(int angle) {
    angle = constrain(angle, 0, 180);
    uint32_t us = _minUs + (uint32_t)((_maxUs - _minUs) * angle) / 180U;
    uint32_t duty = (uint32_t)((uint64_t)us * ((1u<<16) - 1u) / 20000u);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, _channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, _channel);
  }

private:
  int _pin, _minUs, _maxUs;
  ledc_timer_t _timer;
  ledc_channel_t _channel;
};

ServoLite servo1, servo2;

// ---------------- Motor (LEDC) -------------------
static const ledc_timer_t   MOTOR_TIMER  = LEDC_TIMER_0;
static const ledc_channel_t MOTOR_CH_IN1 = LEDC_CHANNEL_0;
static const ledc_channel_t MOTOR_CH_IN2 = LEDC_CHANNEL_1;

inline uint8_t clampDuty(int v) { return (v<0)?0 : (v>255)?255 : (uint8_t)v; }

void motorSetup() {
  // 20 kHz, 8-bit
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_HIGH_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_8_BIT;
  t.timer_num       = MOTOR_TIMER;
  t.freq_hz         = 20000;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.speed_mode = LEDC_HIGH_SPEED_MODE;
  c.intr_type  = LEDC_INTR_DISABLE;
  c.timer_sel  = MOTOR_TIMER;
  c.duty       = 0;
  c.hpoint     = 0;

  c.channel  = MOTOR_CH_IN1;
  c.gpio_num = MOTOR_IN1;
  ledc_channel_config(&c);

  c.channel  = MOTOR_CH_IN2;
  c.gpio_num = MOTOR_IN2;
  ledc_channel_config(&c);
}

void motorStop() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}

// Left: PWM @ IN1, IN2=0
void motorLeft(uint8_t duty) {
  duty = clampDuty(duty);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}

// Right: PWM @ IN2, IN1=0
void motorRight(uint8_t duty) {
  duty = clampDuty(duty);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}

// Giữ tương thích với các nút thủ công
inline void motorLeft()  { motorLeft(DUTY_MANUAL);  }
inline void motorRight() { motorRight(DUTY_MANUAL); }

// ---------------- Servo helpers ------------------
void servoOpen()  { servo1.write(53);  servo2.write(50); }
void servoClose() { servo1.write(116); servo2.write(0);  }

// -------------- Encoder (x4 decode) --------------
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum==0b1101 || sum==0b0100 || sum==0b0010 || sum==0b1011) encoderCount++;
  else if (sum==0b1110 || sum==0b0111 || sum==0b0001 || sum==0b1000) encoderCount--;
  lastEncoded = encoded;
}

// ---------------- Simple HOMING ------------------
// Quay TRÁI chậm đến khi cảm biến ACTIVE (LOW) thì dừng, đặt Bin1, reset encoder.
void resetPosition() {
  servoClose();
  delay(100);

  pinMode(SENSOR_PIN, INPUT_PULLUP); // ACTIVE LOW

  if (SENSOR_INACTIVE()) {
    motorLeft(DUTY_HOME_SLOW);
    while (SENSOR_INACTIVE()) {
      server.handleClient();
      delay(1);
    }
    motorStop();
  }

  noInterrupts(); encoderCount = 0; interrupts();
  setCurrentBin(1, true); // previous = bin cũ, current = 1

  Serial.println("[HOMING] Done (simple, one-direction). At Bin 1.");
}

// Chu trình xả rác khi đã ở đúng bin
static inline void performDumpCycle() {
  delay(2000);      // ổn định
  servoOpen();
  delay(3000);      // mở trong 3s
  servoClose();
}

// --------------- Move to target bin --------------
void rotateToBin(int targetBin) {
  if (targetBin < 1 || targetBin > 4) return;

  // Nếu đang ở đúng bin cần xả: không di chuyển, không đổi previous/current
  if (targetBin == currentBin) {
    performDumpCycle();
    return;
  }

  // Tính hướng ngắn nhất
  int cwDelta  = (targetBin - currentBin + 4) % 4; // right
  int ccwDelta = (currentBin - targetBin + 4) % 4; // left
  long pulsesNeeded;
  bool goLeft;
  if (cwDelta <= ccwDelta) {
    pulsesNeeded = cwDelta * PULSES_PER_BIN;
    goLeft = false; // quay phải
  } else {
    pulsesNeeded = ccwDelta * PULSES_PER_BIN;
    goLeft = true;  // quay trái
  }

  // Reset encoder trước khi di chuyển
  noInterrupts(); encoderCount = 0; interrupts();

  // Bắt đầu ở tốc độ nhanh
  uint8_t duty = DUTY_FAST;
  if (goLeft) motorLeft(duty); else motorRight(duty);

  while (true) {
    server.handleClient();

    long traveled  = labs(encoderCount);
    if (traveled >= pulsesNeeded) break;

    long remaining = pulsesNeeded - traveled;
    uint8_t newDuty =
      (remaining <= THRESH_SLOW) ? DUTY_SLOW :
      (remaining <= THRESH_MED ) ? DUTY_MEDIUM : DUTY_FAST;

    if (newDuty != duty) {
      duty = newDuty;
      if (goLeft) motorLeft(duty); else motorRight(duty);
    }
    delay(1);
  }
  motorStop();

  performDumpCycle();

  // Cập nhật trạng thái: previous = bin cũ, current = target
  setCurrentBin(targetBin, true);
}

// ---------------- HTTP handlers ------------------
void handleRoot() {
  if (!LittleFS.exists("/index.html")) {
    server.send(200, "text/html",
      "<h3>index.html not found in LittleFS</h3>"
      "<p>Put index.html into /data and run uploadfs.</p>");
    return;
  }
  File f = LittleFS.open("/index.html", "r");
  server.streamFile(f, "text/html");
  f.close();
}
void handleEncoder()     { server.send(200, "text/plain", String(encoderCount)); }
void handleMotorLeft()   { motorLeft();  server.send(200, "text/plain", "left"); }
void handleMotorRight()  { motorRight(); server.send(200, "text/plain", "right"); }
void handleMotorStop()   { motorStop();  server.send(200, "text/plain", "stop"); }
void handleServoOpen()   { servoOpen();  server.send(200, "text/plain", "open"); }
void handleServoClose()  { servoClose(); server.send(200, "text/plain", "close"); }

void handleBin1()        { rotateToBin(1); server.send(200, "text/plain", "bin1"); }
void handleBin2()        { rotateToBin(2); server.send(200, "text/plain", "bin2"); }
void handleBin3()        { rotateToBin(3); server.send(200, "text/plain", "bin3"); }
void handleBin4()        { rotateToBin(4); server.send(200, "text/plain", "bin4"); }
void handleReset()       { resetPosition(); server.send(200, "text/plain", "reset"); }

void handleState() {
  String json = "{";
  json += "\"current\":"  + String(currentBin)  + ",";
  json += "\"previous\":" + String(previousBin);
  json += "}";
  server.send(200, "application/json", json);
}

// ---------------- Wi-Fi bootstrap ----------------
bool startWiFiPreferSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);

  WiFi.begin(STA_SSID, STA_PASS);
  Serial.printf("Connecting STA to %s ...\n", STA_SSID);

  const unsigned long TOUT1 = 15000; // 15s để thử DHCP
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < TOUT1) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip  = WiFi.localIP();
    IPAddress gw  = WiFi.gatewayIP();
    IPAddress sn  = WiFi.subnetMask();
    IPAddress dns = WiFi.dnsIP();

    uint32_t gw_u32  = (uint32_t)gw;
    uint32_t sn_u32  = (uint32_t)sn;
    uint32_t dns_u32 = (uint32_t)dns;

    Serial.printf("STA DHCP OK: IP=%s GW=%s SN=%s DNS=%s\n",
      ip.toString().c_str(), gw.toString().c_str(),
      sn.toString().c_str(), dns.toString().c_str());

    // Chỉ khi có gateway & subnet hợp lệ mới ép IP tĩnh cùng subnet
    if (gw_u32 != 0 && sn_u32 != 0) {
      IPAddress desired(gw[0], gw[1], gw[2], DESIRED_LAST_OCTET);

      bool needReconfig = (ip[0] != desired[0] ||
                           ip[1] != desired[1] ||
                           ip[2] != desired[2] ||
                           ip[3] != desired[3]);

      if (needReconfig) {
        Serial.printf("Reconfig static IP to %s ...\n", desired.toString().c_str());
        WiFi.disconnect(true, true);
        delay(200);
        if (dns_u32 == 0) dns = gw; // nếu DNS rỗng, dùng GW làm DNS
        if (!WiFi.config(desired, gw, sn, dns)) {
          Serial.println("WiFi.config() failed");
        }
        WiFi.begin(STA_SSID, STA_PASS);

        const unsigned long TOUT2 = 8000; // 8s cho lần reconnect
        t0 = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - t0) < TOUT2) {
          delay(250);
          Serial.print("#");
        }
        Serial.println();
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("STA Ready. IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
  }

  // Fallback AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP Mode. SSID=%s  PASS=%s  IP=%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  return false;
}

// -------------------- setup/loop ------------------
void setup() {
  Serial.begin(115200);

  // Auto-format if first time / corrupted
  if (!LittleFS.begin(true)) Serial.println("Failed to mount LittleFS (formatted)");
  else                       Serial.println("LittleFS mounted");

  // Wi-Fi: prefer STA(ROBOT), fallback to AP
  bool sta = startWiFiPreferSTA();

  // Motor & servo
  motorSetup();
  motorStop();
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN, 500, 2500, LEDC_TIMER_1, LEDC_CHANNEL_3);
  servoClose();

  // Sensor + Encoder
  pinMode(SENSOR_PIN, INPUT_PULLUP); // ACTIVE LOW
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);

  // Homing đơn giản (1 chiều chậm)
  resetPosition();

  // Routes
  server.on("/",            handleRoot);
  server.on("/encoder",     handleEncoder);
  server.on("/motor/left",  handleMotorLeft);
  server.on("/motor/right", handleMotorRight);
  server.on("/motor/stop",  handleMotorStop);
  server.on("/servo/open",  handleServoOpen);
  server.on("/servo/close", handleServoClose);
  server.on("/bin1",        handleBin1);
  server.on("/bin2",        handleBin2);
  server.on("/bin3",        handleBin3);
  server.on("/bin4",        handleBin4);
  server.on("/reset",       handleReset);
  server.on("/state",       handleState);

  server.begin();
  if (sta) {
    Serial.printf("HTTP server on STA IP: %s:80\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("HTTP server on AP IP:  %s:80\n", WiFi.softAPIP().toString().c_str());
  }
}

void loop() {
  server.handleClient();
}
