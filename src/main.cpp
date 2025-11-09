#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include "driver/ledc.h"

// Chân điều khiển động cơ và encoder
#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define ENCODER_A 34
#define ENCODER_B 35

// Chân servo
#define SERVO1_PIN 21
#define SERVO2_PIN 18

WebServer server(80);

// Biến đếm encoder
volatile long encoderCount = 0;
volatile int lastEncoded = 0;

// Lớp servo sử dụng LEDC của ESP-IDF
class ServoLite {
public:
  bool attach(int pin, int minUs = 500, int maxUs = 2500,
              ledc_timer_t timer = LEDC_TIMER_1, ledc_channel_t channel = LEDC_CHANNEL_2) {
    _pin = pin; _minUs = minUs; _maxUs = maxUs; _timer = timer; _channel = channel;
    // Cấu hình timer 50 Hz độ phân giải 16 bit
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
  int _pin; int _minUs; int _maxUs;
  ledc_timer_t _timer;
  ledc_channel_t _channel;
};

ServoLite servo1, servo2;

// Motor PWM cấu hình LEDC của ESP-IDF
static const ledc_timer_t MOTOR_TIMER  = LEDC_TIMER_0;
static const ledc_channel_t MOTOR_CH_IN1 = LEDC_CHANNEL_0;
static const ledc_channel_t MOTOR_CH_IN2 = LEDC_CHANNEL_1;

void motorSetup() {
  // Cấu hình timer 20 kHz
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

  c.channel = MOTOR_CH_IN1;
  c.gpio_num = MOTOR_IN1;
  ledc_channel_config(&c);

  c.channel = MOTOR_CH_IN2;
  c.gpio_num = MOTOR_IN2;
  ledc_channel_config(&c);
}

void motorStop() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}
void motorLeft() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, 128);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}
void motorRight() {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1, 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN1);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2, 128);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_CH_IN2);
}

// Hàm điều khiển servo cho dễ đọc
void servoOpen() {
  servo1.write(53);
  servo2.write(50);
}
void servoClose() {
  servo1.write(116);
  servo2.write(0);
}

// Hàm ngắt encoder
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum==0b1101 || sum==0b0100 || sum==0b0010 || sum==0b1011) encoderCount++;
  else if (sum==0b1110 || sum==0b0111 || sum==0b0001 || sum==0b1000) encoderCount--;
  lastEncoded = encoded;
}

// Xử lý HTTP
void handleRoot() {
  if (!LittleFS.exists("/index.html")) {
    server.send(200, "text/html", "<h3>index.html not found in LittleFS</h3>");
    return;
  }
  File f = LittleFS.open("/index.html", "r");
  server.streamFile(f, "text/html");
  f.close();
}
void handleEncoder()    { server.send(200, "text/plain", String(encoderCount)); }
void handleMotorLeft()  { motorLeft();  server.send(200, "text/plain", "left");  }
void handleMotorRight() { motorRight(); server.send(200, "text/plain", "right"); }
void handleMotorStop()  { motorStop();  server.send(200, "text/plain", "stop");  }
void handleServoOpen()  { servoOpen();  server.send(200, "text/plain", "open");  }
void handleServoClose() { servoClose(); server.send(200, "text/plain", "close"); }

void setup() {
  Serial.begin(115200);
  // Mount LittleFS
  if (!LittleFS.begin(false)) {
    Serial.println("Failed to mount LittleFS");
  } else {
    Serial.println("LittleFS mounted");
  }
  // WiFi AP
  WiFi.softAP("AI_Trash_Bin", "sgt@since2022");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  // Motor & servo setup
  motorSetup();
  motorStop();
  servo1.attach(SERVO1_PIN);
  // servo2 dùng kênh khác cho 50 Hz
  servo2.attach(SERVO2_PIN, 500, 2500, LEDC_TIMER_1, LEDC_CHANNEL_3);
  servoClose(); // trạng thái ban đầu: đóng
  // Encoder interrupts
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), updateEncoder, CHANGE);
  // Routes
  server.on("/",           handleRoot);
  server.on("/encoder",    handleEncoder);
  server.on("/motor/left", handleMotorLeft);
  server.on("/motor/right",handleMotorRight);
  server.on("/motor/stop", handleMotorStop);
  server.on("/servo/open", handleServoOpen);
  server.on("/servo/close",handleServoClose);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
