// Arduino for the FSM on the Pi: applies the PWM it receives and reports the distance.
// In:  M<left>,<right>  PWM -255..255      X  stop
// Out: D,<cm> every 50 ms  (-1 = no echo, -2 = no signal)
// Stops the motors after 300 ms without M.

#include "Motor.h"
#include "Parameters.h"
#include "MyServo.h"

const int URM_ECHO = 2;
const int SERVO_PIN = 26;
const unsigned long TIMEOUT_MS = 300;
const unsigned long PERIOD_D_MS = 50;
const unsigned long VALID_US = 300000UL;

Motor FRmotor(vitMotor1, dirMotor1, 0);
Motor FLmotor(vitMotor2, dirMotor2, 0);
Motor RRmotor(vitMotor3, dirMotor3, 0);
Motor RLmotor(vitMotor4, dirMotor4, 0);
MyServo frontServo(SERVO_PIN, 20, 160, 0);

volatile unsigned long pulseStart = 0;
volatile unsigned long pulseDuration = 0;
volatile unsigned long lastMeasure = 0;
volatile unsigned long nMeasures = 0;
volatile bool pulseOpen = false;

unsigned long tLastM = 0;
unsigned long tLastD = 0;

void timeEcho() {
  unsigned long now = micros();
  if (digitalRead(URM_ECHO) == LOW) {
    pulseStart = now;
    pulseOpen = true;
  } else if (pulseOpen) {
    pulseDuration = now - pulseStart;
    lastMeasure = now;
    nMeasures++;
    pulseOpen = false;
  }
}

void motors(int left, int right) {
  FRmotor.setvalue(right);
  RRmotor.setvalue(right);
  FLmotor.setvalue(left);
  RLmotor.setvalue(left);
}

int distance() {
  noInterrupts();
  unsigned long p = pulseDuration;
  unsigned long t = lastMeasure;
  unsigned long n = nMeasures;
  interrupts();
  if (n == 0 || micros() - t > VALID_US) return -2;
  unsigned long cm = p / 50;
  return cm > 900 ? -1 : (int)cm;
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'M') {
      int left = Serial.parseInt();
      if (Serial.peek() == ',') Serial.read();
      int right = Serial.parseInt();
      motors(constrain(left, -255, 255), constrain(right, -255, 255));
      tLastM = millis();
    } else if (c == 'X') {
      motors(0, 0);
      tLastM = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  motors(0, 0);
  pinMode(URM_ECHO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(URM_ECHO), timeEcho, CHANGE);
  frontServo.start();
  frontServo.setpos(90);
}

void loop() {
  readCommands();
  if (millis() - tLastM > TIMEOUT_MS) motors(0, 0);
  if (millis() - tLastD >= PERIOD_D_MS) {
    tLastD = millis();
    Serial.print("D,");
    Serial.println(distance());
  }
}
