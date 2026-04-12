// ESP32 WROOM-32D Motor Controller
// I2C slave that receives motor speed commands from a Raspberry Pi 5
// and drives two bidirectional 40A ESCs via PWM.
//
// Protocol (I2C slave @ 0x55):
//   Byte 0: left_speed   (signed, -100..100)
//   Byte 1: right_speed  (signed, -100..100)
//   Byte 2: (optional) heading PID Kp (unsigned, scaled x10)
//   Byte 3: (optional) heading PID Ki (unsigned, scaled x10)
//   Byte 4: (optional) heading PID Kd (unsigned, scaled x10)
//
// ESC PWM range: 1000 us = full reverse, 1500 us = neutral, 2000 us = full forward.

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ---------- Forward declarations ----------
// Declared explicitly so the Arduino IDE does not auto-generate prototypes.
int  speedToPwmUs(int speed);
void onI2CReceive(int numBytes);
void setup();
void loop();

// ---------- Configuration ----------
static const int I2C_SLAVE_ADDR  = 0x55;

static const int LEFT_ESC_PIN    = 16;
static const int RIGHT_ESC_PIN   = 17;

static const int PWM_MIN_US      = 1000;
static const int PWM_NEUTRAL_US  = 1500;
static const int PWM_MAX_US      = 2000;

static const unsigned long WATCHDOG_MS     = 500UL;
static const unsigned long ARM_DURATION_MS = 2000UL;

// ---------- State ----------
Servo leftEsc;
Servo rightEsc;

volatile int           g_leftSpeed  = 0;
volatile int           g_rightSpeed = 0;
volatile bool          g_newCommand = false;
volatile unsigned long g_lastRxMs   = 0;

volatile float g_Kp = 1.0f;
volatile float g_Ki = 0.0f;
volatile float g_Kd = 0.0f;

// Arming state machine
bool           g_armed      = false;
unsigned long  g_armStartMs = 0;

// ---------- Helpers ----------

// Map a speed in [-100, 100] to an ESC PWM pulse width in microseconds.
int speedToPwmUs(int speed) {
  int s = speed;
  if (s >  100) s =  100;
  if (s < -100) s = -100;
  return PWM_NEUTRAL_US + (s * 5);
}

// I2C receive ISR. Keep it short: just latch the new values.
void onI2CReceive(int numBytes) {
  if (numBytes < 2) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return;
  }

  int l = (int)((signed char)Wire.read());
  int r = (int)((signed char)Wire.read());

  g_leftSpeed  = l;
  g_rightSpeed = r;

  if (numBytes >= 5) {
    int kp = (int)((unsigned char)Wire.read());
    int ki = (int)((unsigned char)Wire.read());
    int kd = (int)((unsigned char)Wire.read());
    g_Kp = kp / 10.0f;
    g_Ki = ki / 10.0f;
    g_Kd = kd / 10.0f;
  }

  // Drain any leftover bytes so the next packet starts clean.
  while (Wire.available() > 0) {
    Wire.read();
  }

  g_lastRxMs   = millis();
  g_newCommand = true;
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32 Motor Controller starting...");

  // Allocate timers for ESP32Servo PWM generation.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftEsc.setPeriodHertz(50);
  rightEsc.setPeriodHertz(50);
  leftEsc.attach(LEFT_ESC_PIN,   PWM_MIN_US, PWM_MAX_US);
  rightEsc.attach(RIGHT_ESC_PIN, PWM_MIN_US, PWM_MAX_US);

  // Begin arming: send neutral and record start time (non-blocking)
  leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
  rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
  g_armStartMs = millis();
  Serial.println("Arming ESCs (2s neutral)...");

  // Start I2C slave last, after all other initialization
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);
  Serial.print("I2C slave ready @ 0x");
  Serial.println(I2C_SLAVE_ADDR, HEX);

  g_lastRxMs = millis();
}

void loop() {
  unsigned long now = millis();

  // Arming state machine: hold neutral for ARM_DURATION_MS before accepting commands
  if (!g_armed) {
    leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
    rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
    if ((now - g_armStartMs) >= ARM_DURATION_MS) {
      g_armed = true;
      g_lastRxMs = now;
      Serial.println("ESCs armed.");
    }
    return;
  }

  // Watchdog: if no command received in WATCHDOG_MS, force neutral.
  if ((now - g_lastRxMs) > WATCHDOG_MS) {
    leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
    rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
    return;
  }

  if (g_newCommand) {
    // Snapshot volatile state with interrupts disabled.
    noInterrupts();
    int   ls = g_leftSpeed;
    int   rs = g_rightSpeed;
    float kp = g_Kp;
    float ki = g_Ki;
    float kd = g_Kd;
    g_newCommand = false;
    interrupts();

    int leftUs  = speedToPwmUs(ls);
    int rightUs = speedToPwmUs(rs);

    leftEsc.writeMicroseconds(leftUs);
    rightEsc.writeMicroseconds(rightUs);

    Serial.print("L=");
    Serial.print(ls);
    Serial.print(" R=");
    Serial.print(rs);
    Serial.print("  PWM L=");
    Serial.print(leftUs);
    Serial.print(" R=");
    Serial.print(rightUs);
    Serial.print("  PID Kp=");
    Serial.print(kp, 1);
    Serial.print(" Ki=");
    Serial.print(ki, 1);
    Serial.print(" Kd=");
    Serial.println(kd, 1);
  }

}
