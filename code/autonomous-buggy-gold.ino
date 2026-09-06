#include "WiFiS3.h"
#include <Arduino.h>

// ---------------- PINS ----------------
const int trigPin = 5;
const int echoPin = 8;

const int leftforwardPin = 12;
const int leftbackwardPin = 11;
const int leftenablePin = 9;

const int rightbackwardPin = 7;
const int rightforwardPin = 4;
const int rightenablePin = 6;

const int leftIRPin = 2;
const int rightIRPin = 3;

// -------- ENCODER (CD4040 + shift register) --------
const int latchPin = 3;
const int clockPin = 13;
const int dataPin = 10;
const int reset4040 = A0;

int leftPulses = 0;
int lastPulses = 0;
float currentSpeed = 0;
const float distPerPulse = 2.55;

// ---------------- WIFI ----------------
char ssid[] = "[redacted]";
char pass[] = "[redacted]";

WiFiServer server(5200);
WiFiClient client;

// ---------------- STATE ----------------
bool running = false;
bool obstacleActive = false;

String currentState = "STOPPED";
String lastState = "STOPPED";

// ---------------- TIME ----------------
unsigned long startTime;
unsigned long lastTime;

// ---------------- PID ----------------
float Kp = 2.0;
float Ki = 0.3;
float Kd = 0.1;

float error = 0;
float prevError = 0;
float integral = 0;

// ---------------- SPEED PROFILE ----------------
int timePoints[] = {0, 10, 30, 40, 45};
int speedRefs[] = {20, 30, 10, 20, 10};
int numPoints = 5;

// ---------------- MSE ----------------
float mseSum = 0;
int sampleCount = 0;

// ===================================================
// =================== SETUP ==========================
// ===================================================
void setup() {
  Serial.begin(115200);
  WiFi.beginAP(ssid, pass);
  server.begin();

  pinMode(leftforwardPin, OUTPUT);
  pinMode(leftbackwardPin, OUTPUT);
  pinMode(leftenablePin, OUTPUT);

  pinMode(rightforwardPin, OUTPUT);
  pinMode(rightbackwardPin, OUTPUT);
  pinMode(rightenablePin, OUTPUT);

  pinMode(leftIRPin, INPUT);
  pinMode(rightIRPin, INPUT);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(reset4040, OUTPUT);

  resetCounter();

  startTime = millis();
  lastTime = millis();
}

void loop() {

  // -------- CLIENT --------
  if (!client || !client.connected()) {
    client = server.available();
  }

  if (client && client.available()) {
    char cmd = client.read();

    if (cmd == 'g') {
      running = true;
      startTime = millis();
      resetCounter();
      currentState = "RUNNING";
    }

    if (cmd == 's') {
      running = false;
      stopMotors();
      currentState = "STOPPED";
    }
  }

  if (!running) {
    sendState();
    return;
  }

  // -------- TIME --------
  float t = (millis() - startTime) / 1000.0;

  if (t >= 60) {
    stopMotors();

    float mse = mseSum / sampleCount;

    Serial.print("MSE: ");
    Serial.println(mse);

    if (client) {
      client.print("MSE:");
      client.println(mse);
    }

    while (1);
  }

  // -------- SPEED --------
  updateSpeed();

  float refSpeed = getReferenceSpeed(t);

  float dt = (millis() - lastTime) / 1000.0;
  float control = computePID(refSpeed, currentSpeed, dt);

  int basePWM = 80;
  int pwm = constrain(basePWM + control, 0, 255);

  // -------- OBSTACLE --------
  float distance = readUltrasonic();
  if (distance > 0 && distance < 20) {
    stopMotors();
    obstacleActive = true;
    currentState = "OBSTACLE";
    sendState();
    return;
  }

  // -------- IR LINE FOLLOWING --------
  int leftIR = digitalRead(leftIRPin);
  int rightIR = digitalRead(rightIRPin);

  if (leftIR == HIGH && rightIR == LOW) {
    analogWrite(leftenablePin, pwm);
    analogWrite(rightenablePin, 0);
    digitalWrite(leftforwardPin, HIGH);
    digitalWrite(rightforwardPin, LOW);
  }
  else if (leftIR == LOW && rightIR == HIGH) {
    analogWrite(leftenablePin, 0);
    analogWrite(rightenablePin, pwm);
    digitalWrite(leftforwardPin, LOW);
    digitalWrite(rightforwardPin, HIGH);
  }
  else {
    analogWrite(leftenablePin, pwm);
    analogWrite(rightenablePin, pwm);
    digitalWrite(leftforwardPin, HIGH);
    digitalWrite(rightforwardPin, HIGH);
  }

  // -------- LOGGING --------
  float err = refSpeed - currentSpeed;
  mseSum += err * err;
  sampleCount++;

  if (client) {
    client.print("REF:");
    client.print(refSpeed);
    client.print(",ACT:");
    client.println(currentSpeed);
  }

  lastTime = millis();
  delay(100);
}

// ===================================================
// ================= FUNCTIONS ========================
// ===================================================

void updateSpeed() {
  int pulses = readLeftEncoder();
  int delta = pulses - lastPulses;

  float dt = (millis() - lastTime) / 1000.0;

  float distance = delta * distPerPulse;
  currentSpeed = distance / dt;

  lastPulses = pulses;
}

float computePID(float ref, float actual, float dt) {
  error = ref - actual;

  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  prevError = error;
  return output;
}

float getReferenceSpeed(float t) {
  for (int i = numPoints - 1; i >= 0; i--) {
    if (t >= timePoints[i]) return speedRefs[i];
  }
  return speedRefs[0];
}

long readLeftEncoder() {
  digitalWrite(latchPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(latchPin, LOW);
  return shiftIn(dataPin, clockPin, MSBFIRST);
}

void resetCounter() {
  digitalWrite(reset4040, HIGH);
  delay(10);
  digitalWrite(reset4040, LOW);
}

float readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH, 30000);
  return (duration * 0.0343) / 2;
}

void stopMotors() {
  analogWrite(leftenablePin, 0);
  analogWrite(rightenablePin, 0);
}

void sendState() {
  if (currentState != lastState) {
    if (client) client.println(currentState);
    Serial.println(currentState);
    lastState = currentState;
  }
}
