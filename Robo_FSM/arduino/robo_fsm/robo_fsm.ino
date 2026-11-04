// Arduino for the FSM on the Pi.
// In:  S<left>,<right>  wheel speed in mm/s, per-wheel PI      M<left>,<right>  raw PWM -255..255      X  stop
// Out: D,<cm>,<nFR>,<nFL>,<nRR>,<nRL> every 50 ms  (cm: -1 = no echo, -2 = no signal; n = encoder counts)
// Stops the motors after 300 ms without S or M.
// Grid mission (navigators.py Mission, ported in navigator.cpp), asked by the Pi (pi/arduino_navigator.py):
// In:  N<i>,<j>,<heading 0..3 = E N W S>,<goal i>,<goal j>,<n>,<g mm>   new mission
//      C<exits>,<s mm>  on_node (exits: -1 = None, else bits S=1 L=2 R=4)    B<s mm>,<d mm>  on_blocked
//      F<kind>          on_fault (0 = node missed, 1 = turn failed)
// Out: R,<order>,<prior: 0/1>,<s_next mm>,<exits>,<reason>   answer to each of N, C, B, F

#include "Motor.h"
#include "Parameters.h"
#include "MyServo.h"
#include "navigator.h"

const int URM_ECHO = 2;
const int SERVO_PIN = 26;
const unsigned long TIMEOUT_MS = 300;
const unsigned long PERIOD_D_MS = 50;
const unsigned long PERIOD_PI_MS = 20;
const unsigned long VALID_US = 300000UL;
const unsigned long STOPPED_US = 100000UL;

const float WHEEL_R = 0.0325;
const float COUNTS_PER_REV = 11 * 21.3;
const float U0 = 45.0;
const float KFF = 15.0;
const float KP = 8.0;
const float KI = 60.0;
const int ENC_SIGN[4] = {-1, 1, -1, 1};

Motor motor[4] = {Motor(vitMotor1, dirMotor1, 0), Motor(vitMotor2, dirMotor2, 0),
                  Motor(vitMotor3, dirMotor3, 0), Motor(vitMotor4, dirMotor4, 0)};
const int ENC_A[4] = {Encoder1A, Encoder2A, Encoder3A, Encoder4A};
const int ENC_B[4] = {Encoder1B, Encoder2B, Encoder3B, Encoder4B};
MyServo frontServo(SERVO_PIN, 20, 160, 0);
Mission mission;

volatile long count[4] = {0, 0, 0, 0};
volatile unsigned long lastEdge[4] = {0, 0, 0, 0};
volatile unsigned long period[4] = {0, 0, 0, 0};
volatile int lastDir[4] = {1, 1, 1, 1};

volatile unsigned long pulseStart = 0;
volatile unsigned long pulseDuration = 0;
volatile unsigned long lastMeasure = 0;
volatile unsigned long nMeasures = 0;
volatile bool pulseOpen = false;

float ref[4] = {0, 0, 0, 0};
float integ[4] = {0, 0, 0, 0};
bool closedLoop = false;
unsigned long tLastCmd = 0;
unsigned long tLastD = 0;
unsigned long tLastPI = 0;

void edge(int i) {
  unsigned long now = micros();
  int d = (digitalRead(ENC_A[i]) ? 1 : -1) * ENC_SIGN[i];
  count[i] += d;
  period[i] = now - lastEdge[i];
  lastEdge[i] = now;
  lastDir[i] = d;
}
void edge0() { edge(0); }
void edge1() { edge(1); }
void edge2() { edge(2); }
void edge3() { edge(3); }

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

float wheelSpeed(int i) {
  noInterrupts();
  unsigned long p = period[i];
  unsigned long t = lastEdge[i];
  int d = lastDir[i];
  interrupts();
  if (p == 0 || micros() - t > STOPPED_US) return 0.0;
  return d * 2.0 * PI / (COUNTS_PER_REV * p * 1e-6);
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

void setSides(int left, int right) {
  motor[0].setvalue(right);
  motor[2].setvalue(right);
  motor[1].setvalue(left);
  motor[3].setvalue(left);
}

void stopAll() {
  closedLoop = false;
  for (int i = 0; i < 4; i++) {
    ref[i] = 0;
    integ[i] = 0;
    motor[i].setvalue(0);
  }
}

void controlStep() {
  float dt = PERIOD_PI_MS / 1000.0;
  for (int i = 0; i < 4; i++) {
    if (ref[i] == 0) {
      integ[i] = 0;
      motor[i].setvalue(0);
      continue;
    }
    float e = ref[i] - wheelSpeed(i);
    float u = (ref[i] > 0 ? U0 : -U0) + KFF * ref[i] + KP * e + integ[i];
    bool sat = u > 255 || u < -255;
    if (!sat || (u > 0) != (e > 0)) integ[i] += KI * e * dt;
    motor[i].setvalue(constrain((int)u, -255, 255));
  }
}

void navReply(Order order) {
  long sNext = 0;
  int exits = 0;
  bool known = mission.prior(sNext, exits);
  Serial.print("R,");
  Serial.print(orderName(order));
  Serial.print(known ? ",1," : ",0,");
  Serial.print(sNext);
  Serial.print(",");
  Serial.print(exits);
  Serial.print(",");
  Serial.println(mission.why());
}

void navCommand(char c) {
  if (c == 'N') {
    long v[7];
    for (int k = 0; k < 7; k++) v[k] = Serial.parseInt();
    Node start = {(int8_t)v[0], (int8_t)v[1]}, goal = {(int8_t)v[3], (int8_t)v[4]};
    navReply(mission.begin(start, (int)v[2], goal, (int)v[5], v[6]) ? GO_ON : STOP);
  } else if (c == 'C') {
    int exits = Serial.parseInt();
    long s = Serial.parseInt();
    navReply(mission.onNode(exits, s));
  } else if (c == 'B') {
    long s = Serial.parseInt();
    long d = Serial.parseInt();
    navReply(mission.onBlocked(s, d));
  } else {
    navReply(mission.onFault(Serial.parseInt()));
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'N' || c == 'C' || c == 'B' || c == 'F') {
      navCommand(c);
    } else if (c == 'S' || c == 'M') {
      int left = Serial.parseInt();
      if (Serial.peek() == ',') Serial.read();
      int right = Serial.parseInt();
      tLastCmd = millis();
      if (c == 'S') {
        float wl = left / 1000.0 / WHEEL_R, wr = right / 1000.0 / WHEEL_R;
        ref[0] = ref[2] = wr;
        ref[1] = ref[3] = wl;
        closedLoop = true;
      } else {
        closedLoop = false;
        setSides(constrain(left, -255, 255), constrain(right, -255, 255));
      }
    } else if (c == 'X') {
      stopAll();
      tLastCmd = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  stopAll();
  pinMode(URM_ECHO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(URM_ECHO), timeEcho, CHANGE);
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_B[0]), edge0, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B[1]), edge1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B[2]), edge2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B[3]), edge3, RISING);
  frontServo.start();
  frontServo.setpos(90);
}

void loop() {
  readCommands();
  if (millis() - tLastCmd > TIMEOUT_MS) stopAll();
  if (closedLoop && millis() - tLastPI >= PERIOD_PI_MS) {
    tLastPI = millis();
    controlStep();
  }
  if (millis() - tLastD >= PERIOD_D_MS) {
    tLastD = millis();
    noInterrupts();
    long n[4] = {count[0], count[1], count[2], count[3]};
    interrupts();
    Serial.print("D,");
    Serial.print(distance());
    for (int i = 0; i < 4; i++) {
      Serial.print(",");
      Serial.print(n[i]);
    }
    Serial.println();
  }
}
