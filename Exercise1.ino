#include <Servo.h>

int forwardSpeed = 40; // interval: [-100, 100]
int backwardSpeed = -20; // interval: [-100, 100]
int turningSpeed = 10; // interval: [-100, 100]
int backingTimephotoDetThreshold = 1000; // milliseconds? 1 second?
int turningTimephotoDetThreshold = 1000; // milliseconds? 1 second?
const int speakerPin = 4;
const int redLEDPin = 13;
const int leftEyePin = A3;
const int rightEyePin = A5;
const int leftWhiskerPin = 8;
const int rightWhiskerPin = 9;
int motorLPin = 11;
int motorRPin = 12;

// FSM: 0 = search for mines, 1 = avoid obstacle, 2 = mine detected???
int currentState = 0;
int backingStartTime = 0;
bool isCurrentlyBacking = false;
int turningStartTime = 0;
bool isCurrentlyTurning = false;

Servo ServoL;
Servo ServoR;

// photodetector-related variables
double photoDetThreshold;

// IR-related variables
int distance;
long frequency;

void setup() // Built-in initialization block
{
  tone(speakerPin, 4000, 100); // Play tone for 1 second
  delay(100); // Delay to finish tone
  pinMode(redLEDPin, OUTPUT); 
  pinMode(leftWhiskerPin, INPUT); // Left IR LED & Receiver
  pinMode(rightWhiskerPin, INPUT); // Left IR LED & Receiver
  Serial.begin(9600); // Set data rate to 9600 bps
  ServoL.attach(motorLPin);
  ServoR.attach(motorRPin);
  photoDetThreshold = getAveragePhotoDetThreshold();
}

void loop() // Main loop auto-repeats
{
  // get sensor readings
  bool obstacleDetected = detectObstacle();
  bool mineDetected = detectMine();


  // Finite state machine starts here
  if (currentState == 0) { // search the arena

    if (obstacleDetected) {
      currentState = 1; // avoid the obstacle
    } else if (mineDetected) {
      currentState = 2; // do something with mine
    } else { // if nothing detected, move forward
      ServoL.writeMicroseconds(convertSpeedL(forwardSpeed));
      ServoR.writeMicroseconds(convertSpeedR(forwardSpeed));
    }
  } else if (currentState == 1) { // avoid the obstacle
    stopRobot();
    currentState = 3;
  } else if (currentState == 2) { // do something with mine
    stopRobot();
    currentState = 3;
  } else if (currentState == 3) { // back the robot up
    if (!isCurrentlyBacking) {
      backingStartTime = millis();
      isCurrentlyBacking = true;
    } else {
      if (millis() - backingStartTime < backingTimephotoDetThreshold) {
        ServoL.writeMicroseconds(convertSpeedL(backwardSpeed));
        ServoR.writeMicroseconds(convertSpeedR(backwardSpeed));
      } else {
        isCurrentlyBacking = false;
        currentState = 4; // turn the robot
      }
    }
  } else if (currentState == 4) { // turn the robot
    if (!isCurrentlyTurning) {
      turningStartTime = millis();
      isCurrentlyTurning = true;
    } else {
      if (millis() - turningStartTime < turningTimephotoDetThreshold) {
        ServoL.writeMicroseconds(convertSpeedL(turningSpeed));
        ServoR.writeMicroseconds(convertSpeedR(-turningSpeed));
      } else {
        isCurrentlyTurning = false;
        currentState = 0; // move forward agaidn
      }
    }
  }


  delay(100);
}

void stopRobot() {
  ServoL.writeMicroseconds(convertSpeedL(0));
  ServoR.writeMicroseconds(convertSpeedR(0));
}


bool detectObstacle() {
  return !digitalRead(leftWhiskerPin) || !digitalRead(rightWhiskerPin);
  
}


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

bool detectMine() {
  return (volts(leftEyePin) <= photoDetThreshold) || (volts(rightEyePin) <= photoDetThreshold);
}

int convertSpeedR(int s) {
  return 1500 - s*2;
}

int convertSpeedL(int s) {
  return 1500 + s*2;
}

float volts(int adPin) // Measures volts at adPin
{ // Returns floating point voltage
  return float(analogRead(adPin)) * 5.0 / 1024.0;
}

float getAveragePhotoDetThreshold() {
  float averageVal = volts(leftEyePin) + volts(rightEyePin))/2;
  return averageVal/2;
}