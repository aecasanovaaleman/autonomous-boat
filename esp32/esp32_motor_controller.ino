// ESP32 WROOM-32D Motor Controller
// I2C slave that receives motor speed commands from a Raspberry Pi 5
// and outputs PWM signals to two bidirectional 40A ESCs.
//
// Protocol (I2C slave @ 0x55):
//   Byte 0: left_speed   (int8, -100..100)
//   Byte 1: right_speed  (int8, -100..100)
//   Byte 2: (optional) heading PID Kp  (uint8, scaled x10 -> 0.0..25.5)
//   Byte 3: (optional) heading PID Ki  (uint8, scaled x10 -> 0.0..25.5)
//   Byte 4: (optional) heading PID Kd  (uint8, scaled x10 -> 0.0..25.5)
//
// ESC PWM: 1000us = full reverse, 1500us = neutral, 2000us = full forward

#include <Wire.h>
#include <ESP32Servo.h>

// ---- Configuration ----
static const uint8_t  I2C_SLAVE_ADDR   = 0x55;
static const int      LEFT_ESC_PIN     = 16;
static const int      RIGHT_ESC_PIN    = 17;

static const int      PWM_MIN_US       = 1000;  // full reverse
static const int      PWM_NEUTRAL_US   = 1500;  // stop
static const int      PWM_MAX_US       = 2000;  // full forward

static const uint32_t WATCHDOG_MS      = 500;   // stop motors if no cmd
static const uint32_t ARM_DURATION_MS  = 2000;  // neutral signal on startup

// ---- State ----
Servo leftEsc;
Servo rightEsc;

volatile int8_t  g_leftSpeed   = 0;     // -100..100
volatile int8_t  g_rightSpeed  = 0;     // -100..100
volatile uint32_t g_lastRxMs   = 0;
volatile bool    g_newCommand  = false;

// Heading PID gains (for optional onboard heading PID loop)
volatile float   g_Kp = 1.0f;
volatile float   g_Ki = 0.0f;
volatile float   g_Kd = 0.0f;

// Map int8 speed (-100..100) to PWM microseconds (1000..2000)
static int speedToPwmUs(int8_t speed) {
  if (speed >  100) speed =  100;
  if (speed < -100) speed = -100;
  // 0 -> 1500, +100 -> 2000, -100 -> 1000
  return PWM_NEUTRAL_US + ((int)speed * 5);
}

// I2C receive callback - called from ISR context, keep it short
void onI2CReceive(int numBytes) {
  if (numBytes < 2) {
    // Drain and ignore malformed packets
    while (Wire.available()) Wire.read();
    return;
  }

  g_leftSpeed  = (int8_t)Wire.read();
  g_rightSpeed = (int8_t)Wire.read();

  // Optional PID tuning bytes at offsets 2,3,4
  if (numBytes >= 5) {
    uint8_t kp = Wire.read();
    uint8_t ki = Wire.read();
    uint8_t kd = Wire.read();
    g_Kp = kp / 10.0f;
    g_Ki = ki / 10.0f;
    g_Kd = kd / 10.0f;
  }

  // Drain any extra bytes
  while (Wire.available()) Wire.read();

  g_lastRxMs  = millis();
  g_newCommand = true;
}

void armEscs() {
  leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
  rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
  delay(ARM_DURATION_MS);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Motor Controller starting...");

  // Attach ESCs with full microsecond range
  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);
  leftEsc.attach(LEFT_ESC_PIN,  PWM_MIN_US, PWM_MAX_US);
  rightEsc.attach(RIGHT_ESC_PIN, PWM_MIN_US, PWM_MAX_US);

  Serial.println("Arming ESCs (2s neutral)...");
  armEscs();
  Serial.println("ESCs armed.");

  // Start I2C slave
  Wire.begin((uint8_t)I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Serial.printf("I2C slave ready @ 0x%02X\n", I2C_SLAVE_ADDR);

  g_lastRxMs = millis();
}

void loop() {
  uint32_t now = millis();

  // Watchdog: stop motors if no command received recently
  if ((now - g_lastRxMs) > WATCHDOG_MS) {
    leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
    rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
    return;
  }

  if (g_newCommand) {
    // Snapshot volatile state
    noInterrupts();
    int8_t ls = g_leftSpeed;
    int8_t rs = g_rightSpeed;
    g_newCommand = false;
    interrupts();

    int leftUs  = speedToPwmUs(ls);
    int rightUs = speedToPwmUs(rs);

    leftEsc.writeMicroseconds(leftUs);
    rightEsc.writeMicroseconds(rightUs);

    Serial.printf("L=%4d R=%4d  PWM L=%d R=%d  PID Kp=%.1f Ki=%.1f Kd=%.1f\n",
                  ls, rs, leftUs, rightUs, g_Kp, g_Ki, g_Kd);
  }
}
