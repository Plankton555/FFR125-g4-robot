#include <Servo.h>

const int leftEye = 2;
const int rightEye = 10;
const int frontEye = 3;
const int backEye = 9;
const int leftIRLED = 8;
const int rightIRLED = 4;

Servo servoLeft;
Servo servoRight;

int leftLeftEyeReading;
int leftFrontEyeReading;
int leftRightEyeReading;
int leftBackEyeReading;
int rightLeftEyeReading;
int rightFrontEyeReading;
int rightRightEyeReading;
int rightBackEyeReading;

void setup() {
  setupHardwareConnections();
}

void loop() {
  readSensors();
  debugPrint();
  
  delay(100); 
}

void setupHardwareConnections() {
  pinMode(leftEye, INPUT);
  pinMode(rightEye, INPUT);
  pinMode(frontEye, INPUT);
  pinMode(backEye, INPUT);
  pinMode(leftIRLED, OUTPUT);
  pinMode(rightIRLED, OUTPUT);
  Serial.begin(9600);
}

void debugPrint() {
  Serial.print(leftLeftEyeReading);
  Serial.print(leftFrontEyeReading);
  Serial.print(leftRightEyeReading);
  Serial.print(' ');
  Serial.print(rightLeftEyeReading);
  Serial.print(rightFrontEyeReading);
  Serial.println(rightRightEyeReading);
  Serial.print(' ');
  Serial.print(leftBackEyeReading);
  Serial.print("   ");
  Serial.println(rightBackEyeReading);
  Serial.println();
}

void readSensors() {
  tone(leftIRLED, 38000);
  delay(1);
  leftLeftEyeReading = digitalRead(leftEye);
  leftFrontEyeReading = digitalRead(frontEye);
  leftRightEyeReading = digitalRead(rightEye);
  leftBackEyeReading = digitalRead(backEye);
  noTone(leftIRLED);
  tone(rightIRLED, 38000);
  delay(1);
  rightLeftEyeReading = digitalRead(leftEye);
  rightFrontEyeReading = digitalRead(frontEye);
  rightRightEyeReading = digitalRead(rightEye);
  rightBackEyeReading = digitalRead(backEye);
  noTone(rightIRLED);
}
