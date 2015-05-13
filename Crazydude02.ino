#include <IRLogic.h>
#include <Servo.h>

// Constants

const byte sensorCount = 3;
const byte LEDCount = 2;

const byte sensorPin[sensorCount] = {10, 9, 8}; // left, front, right
const byte LEDPin[LEDCount] = {6, 5};           // left, right

const byte speakerPin = 2;
const byte motorLPin = 11;
const byte motorRPin = 12;

// states for FSM
const int S_SEARCH_ARENA = 1;
const int S_EXIT_SAFEZONE = 2;
const int S_AVOID_WALL = 3;
const int S_GRAB_CYLINDER = 4;
const int S_INVESTIGATE_OBJECT = 5;
const int S_SEARCH_SAFEZONE_HEADING = 6;
const int S_MOVE_TO_SAFEZONE = 7;

// Parameters
const unsigned long UPDATE_INTERVAL = 100; // milliseconds
const unsigned long sensorDelay = 300;     // microseconds ON time, full cycle is 3x


// Variables
Servo servoLeft;
Servo servoRight;

IRLogic sensorState[sensorCount][LEDCount];
boolean sensorDetect[sensorCount];

boolean carryingCylinder = false;

long beaconSearchTime;


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

  for (byte sensorIndex = 0; sensorIndex < sensorCount; sensorIndex++)
    pinMode(sensorPin[sensorIndex], INPUT);

  for (byte LEDIndex = 0; LEDIndex < LEDCount; LEDIndex++)
    pinMode(LEDPin[LEDIndex], OUTPUT);

  // Servo initialization
  servoLeft.attach(motorLPin);
  servoRight.attach(motorRPin);


  Serial.begin(9600);
}

void setupSensors() {
  // Sensor inertia can be changed from default value 10.0 here
  // sensorState[0][0].setInertia(12.0); 
}

void debugPrint() {
  // Debug sensor readings
  // Serial.print(sensorState[0][0].getState());
  
  // Debug beacon detection
  // Serial.print(sensorDetect[0]);
}

void readSensors() {

  unsigned int currentFrequency;

  for (byte sensorIndex = 0; sensorIndex < sensorCount; sensorIndex++) {
    for (byte LEDIndex = 0; LEDIndex < LEDCount ; LEDIndex++) {
      currentFrequency = sensorState[sensorIndex][LEDIndex].getFrequency();
      currentFrequency = constrain(currentFrequency, 38000, 42000);
      tone(LEDPin[LEDIndex], currentFrequency);
      
      // Wait long enough to respond
      delayMicroseconds(sensorDelay);
      
      for (byte writeIndex = 0; writeIndex < sensorCount; writeIndex++)
        sensorDetect[writeIndex] = !digitalRead(sensorPin[writeIndex]);
      noTone(LEDPin[LEDIndex]);
      
      // Wait for duty cycle reasons
      delayMicroseconds(sensorDelay * 2);
      for (byte writeIndex = 0; writeIndex < sensorCount; writeIndex++)
        sensorState[writeIndex][LEDIndex].mark(currentFrequency, sensorDetect[writeIndex]);
    }
  }
  
  // Beacon detection
  for (byte writeIndex = 0; writeIndex < sensorCount; writeIndex++)
    sensorDetect[writeIndex] = !digitalRead(sensorPin[writeIndex]);

}

void moveRobot() {
  // if needed, we can check if the robot still holds the cylinder or not

  // execute FSM
  if (currentState == S_SEARCH_ARENA) {
    // check if we need to go to another state
    if (insideSafeZone) {
    currentState = S_EXIT_SAFEZONE;
    } else if (sensorsDetectedObject) {
      if (sensorsDetectWall) {
        currentState = S_AVOID_WALL;
      } else if (sensorsDetectCylinder) {
        currentState = S_GRAB_CYLINDER;
      } else { // not sure which object
        currentState = S_INVESTIGATE_OBJECT; //???
      }
    }

    performArenaSearch();

  } else if (currentState == S_EXIT_SAFEZONE) {
    performSafezoneExit();
  } else if (currentState == S_AVOID_WALL) {
    performWallAvoidance();
  } else if (currentState == S_GRAB_CYLINDER) {
    performCylinderGrabbing();
  } else if (currentState == S_INVESTIGATE_OBJECT) {
    // behaviour
    // ???
  } else if (currentState == S_SEARCH_SAFEZONE_HEADING) {
    performBeaconSearch();
    // behaviour
    currentState = S_MOVE_TO_SAFEZONE;
  } else if (currentState == S_MOVE_TO_SAFEZONE) {

    // if inside safezone, currentState = S_EXIT_SAFEZONE

    // after a certain time, currentState = S_SEARCH_SAFEZONE_HEADING
  } else {
    // This should never happen
  }
}

void performArenaSearch() {
  moveRobot(80);
}

void performSafezoneExit() {
  moveRobot(-80);
  delayMicroseconds(2000);
  turnRobot(-50, 50);
  currentState = S_SEARCH_ARENA;
}

void performWallAvoidance() {
  moveRobot(-80);
  delayMicroseconds(1000);
  turnRobot(-50, 50);
  currentState = S_SEARCH_ARENA;
}

void performCylinderGrabbing() {
  // how do we do this one???
  /*
  moveRobot(-80);
  delayMicroseconds(1000);
  turnRobot(-50, 50);
  currentState = S_SEARCH_SAFEZONE_HEADING;
  */
}

void performBeaconSearch() {
  beaconSearchIntensity = 0; // should be global
  beaconSearchTime = 0; // should be global
  turnRobot(-20, 20);
}


// *********************************************************''

void moveRobot(int speed) {
  servoLeft.writeMicroseconds(convertSpeedL(speed));
  servoRight.writeMicroseconds(convertSpeedR(speed));
}

void turnRobot(int leftWheelSpeed, int rightWheelSpeed) {
  servoLeft.writeMicroseconds(convertSpeedL(leftWheelSpeed));
  servoRight.writeMicroseconds(convertSpeedR(rightWheelSpeed));
}

int convertSpeedR(int s) {
  return 1500 - s * 2;
}

int convertSpeedL(int s) {
  return 1500 + s * 2;
}
