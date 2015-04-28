#include <Servo.h>

// Pin assignments

const int speakerPin = 4;
const int redLEDPin = 13;
const int leftIRLEDPin = 5;
const int rightIRLEDPin = 6;
const int leftEyePin = A3;
const int rightEyePin = A5;
const int leftWhiskerPin = 8;
const int rightWhiskerPin = 9;
const int motorLPin = 11;
const int motorRPin = 12;


// Motion parameters

const int forwardSpeed = 190;
const int backwardSpeed = -20;
const int turningSpeed = 20;
const unsigned long backingTimeThreshold = 1000;
const unsigned long turningTimeThreshold = 1500;


// Finite-state machine

int currentState = 0;
unsigned long backingStartTime = 0;
unsigned long turningStartTime = 0;
int turningDirection = 1;


// Photodetector

float leftEyeAvg;
float rightEyeAvg;
const float decayRate = 1 - 1 / 20.0;
const float detectThreshold = 0.6;
bool leftDetect;
bool rightDetect;


// Servos

Servo ServoL;
Servo ServoR;


void setup() {

// Startup tone
  tone(speakerPin, 4000, 100);
  delay(100);

// Pin mode assignments
  pinMode(redLEDPin, OUTPUT);
  pinMode(leftIRLEDPin, OUTPUT);
  pinMode(rightIRLEDPin, OUTPUT); 
  pinMode(leftWhiskerPin, INPUT);
  pinMode(rightWhiskerPin, INPUT);

// Servo initialization
  ServoL.attach(motorLPin);
  ServoR.attach(motorRPin);

// Eye lamps on
  digitalWrite(leftIRLEDPin, HIGH);
  digitalWrite(rightIRLEDPin, HIGH);
}


void loop() {

// Sensor readings
  bool obstacleDetected = detectObstacle();
  updateEyes();
  bool mineDetected = detectMine();


// Finite state machine
// State 0: drive forward
  if (currentState == 0) {
    if (obstacleDetected) {
      currentState = 1;
      turningDirection = digitalRead(rightWhiskerPin) - digitalRead(leftWhiskerPin);
      if (turningDirection == 0) turningDirection = -1;
    }
    else if (mineDetected) {
      currentState = 2;
      turningDirection = 1;      
      if (rightDetect) turningDirection = -1;
    } 
    else {
      ServoL.writeMicroseconds(convertSpeedL(forwardSpeed));
      ServoR.writeMicroseconds(convertSpeedR(forwardSpeed));
    }
  }

// State 1: stop for obstacle 
  else if (currentState == 1) {
    stopRobot();
    currentState = 3;
    backingStartTime = millis();
  }

// State 2: stop for mine and play tone 
  else if (currentState == 2) {
    stopRobot();
    currentState = 3;
    tone(speakerPin, 4000, 40); delay(80);
    tone(speakerPin, 8000, 40); delay(80);
    tone(speakerPin, 4000, 250); delay(250);
    backingStartTime = millis();
  }

// State 3: drive backward
  else if (currentState == 3) {
    if (millis() - backingStartTime < backingTimeThreshold) {
      ServoL.writeMicroseconds(convertSpeedL(backwardSpeed));
      ServoR.writeMicroseconds(convertSpeedR(backwardSpeed));
    } 
    else {
      currentState = 4;
      turningStartTime = millis();
    }
  }

// State 4: turn away 
  else if (currentState == 4) {
    if (millis() - turningStartTime < turningTimeThreshold) {
      ServoL.writeMicroseconds(convertSpeedL(turningSpeed*turningDirection));
      ServoR.writeMicroseconds(convertSpeedR(-turningSpeed*turningDirection));
    } 
    else {
      currentState = 0;
    }
  }

// IR receiver visibility
  tone(redLEDPin, 38000);
  delay(50);
  noTone(redLEDPin);
}

void stopRobot() {
  ServoL.writeMicroseconds(convertSpeedL(0));
  ServoR.writeMicroseconds(convertSpeedR(0));
}


bool detectObstacle() {
  return !digitalRead(leftWhiskerPin) || !digitalRead(rightWhiskerPin);
}

void updateEyes() {
  float leftEye = float(analogRead(leftEyePin));
  float rightEye = float(analogRead(rightEyePin));
  leftEyeAvg = leftEyeAvg * decayRate + leftEye * (1 - decayRate);
  rightEyeAvg = rightEyeAvg * decayRate + rightEye * (1 - decayRate);
  leftDetect = leftEyeAvg * detectThreshold > leftEye;
  rightDetect = rightEyeAvg * detectThreshold > rightEye;
}

bool detectMine() {
  return (leftDetect || rightDetect);
}

int convertSpeedR(int s) {
  return 1500 - s*2;
}

int convertSpeedL(int s) {
  return 1500 + s*2;
}
