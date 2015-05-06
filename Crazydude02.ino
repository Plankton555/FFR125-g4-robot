#include <IRLogic.h>
#include <Servo.h>

// Constants
const int speakerPin = 2;
const int leftEye = 10;
const int rightEye = 8;
const int frontEye = 9;
const int backEye = 9;
const int leftIRLED = 6;
const int rightIRLED = 5;
const int motorLPin = 11;
const int motorRPin = 12;

// Parameters
const unsigned long UPDATE_INTERVAL = 100; // milliseconds


// Variables
Servo servoLeft;
Servo servoRight;

float initWeight = 10;
IRLogic leftLeftEye(initWeight);
IRLogic leftFrontEye(initWeight);
IRLogic leftRightEye(initWeight);
IRLogic leftBackEye(initWeight);
IRLogic rightLeftEye(initWeight);
IRLogic rightFrontEye(initWeight);
IRLogic rightRightEye(initWeight);
IRLogic rightBackEye(initWeight);

long leftLeftEyeReading;
long leftFrontEyeReading;
long leftRightEyeReading;
long leftBackEyeReading;
long rightLeftEyeReading;
long rightFrontEyeReading;
long rightRightEyeReading;
long rightBackEyeReading;

bool carryingCylinder = false;

double roam_forwardSpeed = 50;
double roam_currentSpeed = roam_forwardSpeed;
double roam_variance = 20;
double roam_change = 5;

unsigned long timeSincePrint;


/**
 * Called once before all the looping starts
 **/
void setup() {
  // Startup tone
  tone(speakerPin, 4000, 100);
  delay(100);

  setupHardwareConnections();
  setupSensors();
  timeSincePrint = millis();
}

/**
 * The actual loop function called by Arduino
 **/
void loop() {

  readSensors();
  moveRobot();

  
  // do delay unless execution has taken too much time
  unsigned long timeSpent = millis() - timeSincePrint;
  if (timeSpent < UPDATE_INTERVAL) {
    //delay(UPDATE_INTERVAL - timeSpent); 
  } else {
    // overdue, return from function

    debugPrint();
    timeSincePrint = millis();
  }
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

  // Servo initialization
  servoLeft.attach(motorLPin);
  servoRight.attach(motorRPin);


  Serial.begin(9600);
}

void setupSensors() {
  /*
  for (long freq=1; freq<40000; freq += 7) {
    leftLeftEye.mark(freq, freq%2 == 0);
    leftFrontEye.mark(freq, freq%2 == 0);
    leftRightEye.mark(freq, freq%2 == 0);
    leftBackEye.mark(freq, freq%2 == 0);
    rightLeftEye.mark(freq, freq%2 == 0);
    rightFrontEye.mark(freq, freq%2 == 0);
    rightRightEye.mark(freq, freq%2 == 0);
    rightBackEye.mark(freq, freq%2 == 0);
  }*/
}

void debugPrint() {
  Serial.print(leftLeftEyeReading);
  //Serial.print(leftFrontEyeReading);
  //Serial.print(leftRightEyeReading);
  Serial.print(' ');
  Serial.print(rightLeftEyeReading);
  //Serial.print(rightFrontEyeReading);
  //Serial.println(rightRightEyeReading);
  Serial.println();
}

void readSensors() {
  long currentFrequency = long(leftLeftEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(leftIRLED, currentFrequency);
  delay(1);
  leftLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  leftFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  leftRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(leftIRLED);

  currentFrequency = long(rightLeftEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(rightIRLED, currentFrequency);
  delay(1);
  rightLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  rightFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  rightRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(rightIRLED);


  currentFrequency = long(leftFrontEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(leftIRLED, currentFrequency);
  delay(1);
  leftLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  leftFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  leftRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(leftIRLED);

  currentFrequency = long(rightFrontEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(rightIRLED, currentFrequency);
  delay(1);
  rightLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  rightFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  rightRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(rightIRLED);


  currentFrequency = long(leftRightEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(leftIRLED, currentFrequency);
  delay(1);
  leftLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  leftFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  leftRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(leftIRLED);

  currentFrequency = long(rightRightEye.getState());
  currentFrequency = constrain(currentFrequency, 38000, 42000);
  tone(rightIRLED, currentFrequency);
  delay(1);
  rightLeftEye.mark(currentFrequency, !digitalRead(leftEye));
  rightFrontEye.mark(currentFrequency, !digitalRead(frontEye));
  rightRightEye.mark(currentFrequency, !digitalRead(rightEye));
  noTone(rightIRLED);



  leftLeftEyeReading = long(leftLeftEye.getState());
  leftFrontEyeReading = long(leftFrontEye.getState());
  leftRightEyeReading = long(leftRightEye.getState());
  rightLeftEyeReading = long(rightLeftEye.getState());
  rightFrontEyeReading = long(rightFrontEye.getState());
  rightRightEyeReading = long(rightRightEye.getState());
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
  double rndNr = ((double)random(0, 100))/100;
  double rndChange = (rndNr-0.5)*2*roam_change; // change seed here
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