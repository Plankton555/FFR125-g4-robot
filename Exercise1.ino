#include <Servo.h>

int forwardSpeed = 20; // interval: [-100, 100]
int backwardSpeed = -20; // interval: [-100, 100]
int turningSpeed = 10; // interval: [-100, 100]
int backingTimeThreshold = 1000; // milliseconds? 1 second?
int turningTimeThreshold = 1000; // milliseconds? 1 second?
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

int distance;
long frequency;

void setup() // Built-in initialization block
{
  tone(4, 3000, 1000); // Play tone for 1 second
  delay(1000); // Delay to finish tone
  pinMode(10, INPUT); 
  pinMode(9, OUTPUT); // Left IR LED & Receiver
  Serial.begin(9600); // Set data rate to 9600 bps
  ServoL.attach(motorLPin);
  ServoR.attach(motorRPin);
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
      ServoL.writeMicroseconds(convertSpeed(forwardSpeed));
      ServoR.writeMicroseconds(convertSpeed(forwardSpeed));
    }
  } else if (currentState == 1) { // avoid the obstacle
    stopRobot();
    // bla bla
  } else if (currentState == 2) { // do something with mine
    stopRobot();
    // blabla
  } else if (currentState == 3) { // back the robot up
    if (!isCurrentlyBacking) {
      backingStartTime = millis();
      isCurrentlyBacking = true;
    } else {
      if (millis() - backingStartTime < backingTimeThreshold) {
        ServoL.writeMicroseconds(convertSpeed(backwardSpeed));
        ServoR.writeMicroseconds(convertSpeed(backwardSpeed));
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
      if (millis() - turningStartTime < turningTimeThreshold) {
        ServoL.writeMicroseconds(convertSpeed(turningSpeed));
        ServoR.writeMicroseconds(convertSpeed(-turningSpeed));
      } else {
        isCurrentlyTurning = false;
        currentState = 0; // move forward agaidn
      }
    }
  }


  delay(100);
}

void stopRobot() {
  ServoL.writeMicroseconds(convertSpeed(0));
  ServoR.writeMicroseconds(convertSpeed(0));
}


bool detectObstacle() {
  // this code uses IR sensor
  distance=0;
  for(frequency=38000; frequency<42000;frequency+=100){
    int irLeft = irDetect(9, 10, frequency); // Check for object
    distance+=irLeft;
  }
  Serial.println(distance);
  //return (distance < 20);
  return false;
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
  return false;
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
  return false;
}

int convertSpeed(int s) {
  return s*2 + 1500;
}