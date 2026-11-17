#include <Servo.h>

/* ---- pins ---------------------------------------------------------------- */
const uint8_t PIN_SERVO = 12;
const uint8_t PIN_TRIG  = 11;
const uint8_t PIN_ECHO  = 10;

/* ---- sweep --------------------------------------------------------------- */
const int ANGLE_MIN = 15;                 // servo limits, degrees
const int ANGLE_MAX = 165;

struct Speed { uint8_t stepDeg; uint8_t stepMs; };
const Speed SPEEDS[3] = {
  {1, 60},                                // 1 slow:   best detail
  {1, 35},                                // 2 normal
  {2, 35}                                 // 3 fast:   2 degrees per step
};
uint8_t speedIdx = 1;

/* ---- ranging ------------------------------------------------------------- */
const int           MAX_RANGE_CM    = 200;
const int           MIN_RANGE_CM    = 2;
const unsigned long US_PER_CM       = 58UL;      // 343 m/s, out and back
const unsigned long ECHO_TIMEOUT_US = MAX_RANGE_CM * US_PER_CM + 400UL;
const unsigned long MODULE_BUSY_US  = 40000UL;   // HC-SR04 holds ECHO high up
                                                 // to ~38 ms when nothing echoes
const int NO_ECHO = 999;                         // sorts as "very far"

Servo head;
int  angle  = ANGLE_MIN;
int  dir    = 1;
bool paused = false;
unsigned long lastStep = 0;

/* last three readings and the angles they were taken at */
int histCm[3]  = {NO_ECHO, NO_ECHO, NO_ECHO};
int histAng[3] = {ANGLE_MIN, ANGLE_MIN, ANGLE_MIN};

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  head.attach(PIN_SERVO);
  head.write(angle);
  delay(600);                             // let the head reach the start

  Serial.println(F("# sonar radar v2 ready"));
}

/* One trigger pulse, one echo. Returns cm, or NO_ECHO. */
int pingOnce() {
  /* If the last ping found nothing, the module may still be holding ECHO high.
     A trigger sent now would be ignored, so wait for the line to drop first. */
  unsigned long t0 = micros();
  while (digitalRead(PIN_ECHO) == HIGH && micros() - t0 < MODULE_BUSY_US) { }

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);                  // the HC-SR04 wants a 10 us trigger
  digitalWrite(PIN_TRIG, LOW);

  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NO_ECHO;

  int cm = (int)(us / US_PER_CM);
  if (cm < MIN_RANGE_CM || cm > MAX_RANGE_CM) return NO_ECHO;
  return cm;
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'p' || c == 'P') {
      paused = !paused;
      Serial.println(paused ? F("# sweep paused") : F("# sweep resumed"));
    } else if (c >= '1' && c <= '3') {
      speedIdx = (uint8_t)(c - '1');
      Serial.print(F("# speed "));
      Serial.println(speedIdx + 1);
    }
  }
}

void loop() {
  readCommands();

  unsigned long now = millis();
  if (paused || now - lastStep < SPEEDS[speedIdx].stepMs) return;
  lastStep = now;

  /* The head has been resting at `angle` since the last step, so measure now. */
  histCm[0]  = histCm[1];   histCm[1]  = histCm[2];   histCm[2]  = pingOnce();
  histAng[0] = histAng[1];  histAng[1] = histAng[2];  histAng[2] = angle;

  /* Running median over three neighbouring angles. One false echo in empty
     space or one dropout inside an object cannot move a median. The result
     belongs to the middle angle, so edges of objects stay where they are. */
  int cm = median3(histCm[0], histCm[1], histCm[2]);

  Serial.print(histAng[1]);
  Serial.print(',');
  Serial.println(cm == NO_ECHO ? -1 : cm);

  /* move on for the next reading */
  angle += dir * SPEEDS[speedIdx].stepDeg;
  if (angle >= ANGLE_MAX) { angle = ANGLE_MAX; dir = -1; }
  if (angle <= ANGLE_MIN) { angle = ANGLE_MIN; dir =  1; }
  head.write(angle);
}
