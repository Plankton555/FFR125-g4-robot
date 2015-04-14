#include <Servo.h>

const int forwardSpeed = 60; // interval: [-100, 100]
const int backwardSpeed = -20; // interval: [-100, 100]
const int turningSpeed = 10; // interval: [-100, 100]
const unsigned long backingTimeThreshold = 1000; // milliseconds?
const unsigned long turningTimeThreshold = 2000; // milliseconds?
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

// FSM: 0 = search for mines, 1 = avoid obstacle, 2 = mine detected???
int currentState = 0;
unsigned long backingStartTime = 0;
bool isCurrentlyBacking = false;
unsigned long turningStartTime = 0;
bool isCurrentlyTurning = false;

Servo ServoL;
Servo ServoR;
int turningDirection = 1;

// photodetector-related variables
float photoDetThreshold;
float leftEyeAvg;
float rightEyeAvg;
float decayRate = 1 - 1 / 20.0;
float detectThreshold = 0.5;
bool leftDetect;
bool rightDetect;

// IR-related variables
int distance;
long frequency;

void setup() // Built-in initialization block
{
  tone(speakerPin, 4000, 100); // Play tone for 1 second
  delay(100); // Delay to finish tone
  pinMode(redLEDPin, OUTPUT);
  pinMode(leftIRLEDPin, OUTPUT);
  pinMode(rightIRLEDPin, OUTPUT); 
  pinMode(leftWhiskerPin, INPUT); // Left IR LED & Receiver
  pinMode(rightWhiskerPin, INPUT); // Left IR LED & Receiver
  Serial.begin(9600); // Set data rate to 9600 bps
  ServoL.attach(motorLPin);
  ServoR.attach(motorRPin);
  photoDetThreshold = getAveragePhotoDetThreshold();
  digitalWrite(leftIRLEDPin, HIGH);
  digitalWrite(rightIRLEDPin, HIGH);
  Serial.println("Searching");
}

void loop() // Main loop auto-repeats
{
  // get sensor readings
  bool obstacleDetected = detectObstacle();
  updateEyes();
  bool mineDetected = detectMine();


  // Finite state machine starts here
  if (currentState == 0) { // search the arena

    if (obstacleDetected) {
      currentState = 1; // avoid the obstacle
      
      Serial.println("Bump");
      
      turningDirection = digitalRead(rightWhiskerPin) - digitalRead(leftWhiskerPin);
      if (turningDirection == 0){
        turningDirection = -1;
      }
      
    } 
    else if (mineDetected) {
      currentState = 2; // do something with mine

      turningDirection = 1;      
      if (leftDetect) {
        Serial.println("Left");
      }
      if (rightDetect) {
        turningDirection = -1;
        Serial.println("Right");
      }
        
    } 
    else { // if nothing detected, move forward
      ServoL.writeMicroseconds(convertSpeedL(forwardSpeed));
      ServoR.writeMicroseconds(convertSpeedR(forwardSpeed));
    }
  } 
  else if (currentState == 1) { // avoid the obstacle
    stopRobot();
    currentState = 3;
    Serial.println("Backing");
    backingStartTime = millis();
  } 
  else if (currentState == 2) { // do something with mine
    stopRobot();
    currentState = 3;

    Serial.println("Backing");
    tone(speakerPin, 4000, 40);
    delay(80);
    tone(speakerPin, 8000, 40);
    delay(80);
    tone(speakerPin, 4000, 250);
    delay(250);
    tone(speakerPin, 8000, 40);

    backingStartTime = millis();
  } 
  else if (currentState == 3) { // back the robot up
    if (millis() - backingStartTime < backingTimeThreshold) {
      ServoL.writeMicroseconds(convertSpeedL(backwardSpeed));
      ServoR.writeMicroseconds(convertSpeedR(backwardSpeed));
    } 
    else {
      isCurrentlyBacking = false;
      currentState = 4; // turn the robot
      Serial.println("Turning");
      turningStartTime = millis();
    }

  } 
  else if (currentState == 4) { // turn the robot
    if (millis() - turningStartTime < turningTimeThreshold) {
      ServoL.writeMicroseconds(convertSpeedL(turningSpeed*turningDirection));
      ServoR.writeMicroseconds(convertSpeedR(-turningSpeed*turningDirection));
    } 
    else {
      isCurrentlyTurning = false;
      currentState = 0; // move forward agaidn
      Serial.println("Searching");
    }
  }


  delay(50);
}

void stopRobot() {
  ServoL.writeMicroseconds(convertSpeedL(0));
  ServoR.writeMicroseconds(convertSpeedR(0));
}


bool detectObstacle() {
  return !digitalRead(leftWhiskerPin) || !digitalRead(rightWhiskerPin);

}

void updateEyes() {
  float leftEye = volts(leftEyePin);
  float rightEye = volts(rightEyePin);
  leftEyeAvg = leftEyeAvg * decayRate + leftEye * (1 - decayRate);
//  Serial.println(leftEye);
 // Serial.println(leftEyeAvg);
 // Serial.println(' ');
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

float volts(int adPin) // Measures volts at adPin
{ // Returns floating point voltage
  return float(analogRead(adPin));// * 5.0 / 1024.0;
}

float getAveragePhotoDetThreshold() {
  float averageVal = (volts(leftEyePin) + volts(rightEyePin))/2;
  return averageVal/2;
}

/*
void detectObstacleIR() {
  // this code is not used currently
  distance=0;
  for(frequency=38000; frequency<42000;frequency+=100){
    int irLeft = irDetect(9, 10, frequency); // Check for object
    distance+=irLeft;
  }
  Serial.println(distance);
  //return (distance < 20);
  //return false;
}
// IR Object Detection Function
int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8); // IRLED 38 kHz for at least 1 ms
  delay(1); // Wait 1 ms
  int ir = digitalRead(irReceiverPin); // IR receiver -> ir variable
  delay(1); // Down time before recheck
  return ir; // Return 1 no detect, 0 detect
}
*/

