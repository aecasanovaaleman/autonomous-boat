// ESP32 WROOM-32D Motor Controller
// Receives motor speed commands from a Raspberry Pi 5 via USB Serial (UART)
// and drives two bidirectional 40A ESCs via PWM.
//
// Protocol (USB Serial @ 115200 baud):
//   Byte 0: left_speed   (signed int8, -100..100)
//   Byte 1: right_speed  (signed int8, -100..100)
//
// ESC PWM range: 1000 us = full reverse, 1500 us = neutral, 2000 us = full forward.

#include <Arduino.h>
#include <ESP32Servo.h>

// ---------- Forward declarations ----------
int  speedToPwmUs(int speed);
void setup();
void loop();

// ---------- Configuration ----------
static const int LEFT_ESC_PIN    = 25;
static const int RIGHT_ESC_PIN   = 26;

static const int PWM_MIN_US      = 1000;
static const int PWM_NEUTRAL_US  = 1500;
static const int PWM_MAX_US      = 2000;

static const unsigned long WATCHDOG_MS     = 500UL;
static const unsigned long ARM_DURATION_MS = 2000UL;

// ---------- State ----------
Servo leftEsc;
Servo rightEsc;

int           g_leftSpeed  = 0;
int           g_rightSpeed = 0;
bool          g_newCommand = false;
unsigned long g_lastRxMs   = 0;

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

  Serial.println("USB Serial ready @ 115200 baud");
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
    // Drain any bytes received during arming
    while (Serial.available() > 0) {
      Serial.read();
    }
    return;
  }

  // Read serial commands: expect exactly 2 bytes [left_speed, right_speed]
  while (Serial.available() >= 2) {
    int l = (int)((signed char)Serial.read());
    int r = (int)((signed char)Serial.read());

    g_leftSpeed  = l;
    g_rightSpeed = r;
    g_lastRxMs   = now;
    g_newCommand = true;
  }

  // Watchdog: if no command received in WATCHDOG_MS, force neutral.
  if ((now - g_lastRxMs) > WATCHDOG_MS) {
    leftEsc.writeMicroseconds(PWM_NEUTRAL_US);
    rightEsc.writeMicroseconds(PWM_NEUTRAL_US);
    return;
  }

  if (g_newCommand) {
    int ls = g_leftSpeed;
    int rs = g_rightSpeed;
    g_newCommand = false;

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
    Serial.println(rightUs);
  }
}
