#include <Servo.h>

const int leftEye = 2;
const int rightEye = 10;
const int frontEye = 3;
const int backEye = 9;
const int leftIRLED = 8;
const int rightIRLED = 4;

Servo ServoL;
Servo ServoR;

int LLDetect;
int LFDetect;
int LRDetect;
int LBDetect;
int RLDetect;
int RFDetect;
int RRDetect;
int RBDetect;

void setup() {
  pinMode(leftEye, INPUT);
  pinMode(rightEye, INPUT);
  pinMode(frontEye, INPUT);
  pinMode(backEye, INPUT);
  pinMode(leftIRLED, OUTPUT);
  pinMode(rightIRLED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  tone(leftIRLED, 38000);
  delay(1);
  LLDetect = digitalRead(leftEye);
  LFDetect = digitalRead(frontEye);
  LRDetect = digitalRead(rightEye);
  LBDetect = digitalRead(backEye);
  noTone(leftIRLED);
  tone(rightIRLED, 38000);
  delay(1);
  RLDetect = digitalRead(leftEye);
  RFDetect = digitalRead(frontEye);
  RRDetect = digitalRead(rightEye);
  RBDetect = digitalRead(backEye);
  noTone(rightIRLED);
  
  Serial.print(LLDetect);
  Serial.print(LFDetect);
  Serial.print(LRDetect);
  Serial.print(' ');
  Serial.print(RLDetect);
  Serial.print(RFDetect);
  Serial.println(RRDetect);
  Serial.print(' ');
  Serial.print(LBDetect);
  Serial.print("   ");
  Serial.println(RBDetect);
  Serial.println();
  delay(100);
  
}
