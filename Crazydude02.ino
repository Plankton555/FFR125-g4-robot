#include <Servo.h>

// Constants
const int speakerPin = 2;
const int leftEye = 2;
const int rightEye = 10;
const int frontEye = 3;
const int backEye = 9;
const int leftIRLED = 8;
const int rightIRLED = 4;

// Parameters
const unsigned long UPDATE_INTERVAL = 100; // milliseconds


// Variables
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

bool carryingCylinder = false;

double roam_forwardSpeed = 50;
double roam_currentSpeed = roam_forwardSpeed;
double roam_variance = 10;
double roam_change = 1;


/**
 * Called once before all the looping starts
 **/
void setup() {
  // Startup tone
  tone(speakerPin, 4000, 100);
  delay(100);

  setupHardwareConnections();
}

/**
 * The actual loop function called by Arduino
 **/
void loop() {
  unsigned long startTime = millis();

  readSensors();
  moveRobot();

  debugPrint();
  
  // do delay unless execution has taken too much time
  unsigned long timeSpent = millis() - startTime;
  if (timeSpent < UPDATE_INTERVAL) {
    delay(UPDATE_INTERVAL - timeSpent); 
  } /*else {
    // overdue, return from function
  }*/
}


// *******************************************************************

/**
 * Sets up hardware connections, such as pins and sensors
 **/
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

void moveRobot() {
  if (carryingCylinder) {
    // Find safe zone and return cylinder to it

  } else {
    // Roam arena and find a cylinder
    performRoamingBehavior();
  }
}

void performRoamingBehavior() {
  double rndChange = (rand()-0.5)*2*roam_change; // change seed here
  roam_currentSpeed += rndChange;
  // clamp the speed
  if (roam_currentSpeed < roam_forwardSpeed - roam_variance) {
    roam_currentSpeed = roam_forwardSpeed - roam_variance;
  } else if (roam_currentSpeed > roam_forwardSpeed + roam_variance) {
    roam_currentSpeed = roam_forwardSpeed + roam_variance;
  }

  // make sure keep approximately constant speed
  double diff = roam_currentSpeed - roam_forwardSpeed;
  servoLeft.writeMicroseconds(convertSpeedL(roam_forwardSpeed + diff));
  servoRight.writeMicroseconds(convertSpeedR(roam_forwardSpeed - diff));
}



int convertSpeedR(int s) {
  return 1500 - s*2;
}

int convertSpeedL(int s) {
  return 1500 + s*2;
}

void stopRobot() {
  servoLeft.writeMicroseconds(convertSpeedL(0));
  servoRight.writeMicroseconds(convertSpeedR(0));
}