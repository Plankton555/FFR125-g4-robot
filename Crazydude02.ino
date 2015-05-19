#include <IRLogic.h>
#include <Servo.h>

/**
 * Pin assignments
 **/

const byte sensorCount = 3;
const byte LEDCount = 2;

const byte sensorPin[sensorCount] = {8, 3, 9}; // left, front, right
const byte LEDPin[LEDCount] = {5, 4};           // left, right
const byte rearSensorPin = 10;
const byte floorSensorPin = A3;

const byte speakerPin = 2;
const byte motorLPin = 11;
const byte motorRPin = 12;
const byte redLEDPin = 13;


/**
 * Behaviour state machine
 **/

const byte S_SEARCH_ARENA = 1;
const byte S_EXIT_SAFEZONE = 2;
const byte S_AVOID_WALL = 3;
const byte S_GRAB_CYLINDER = 4;
const byte S_INVESTIGATE_OBJECT = 5;
const byte S_SEARCH_SAFEZONE = 6;
const byte S_MOVE_TO_SAFEZONE = 7;
byte currentState = 1;

int actionCounter = 0;
bool insideSafeZone = false;


/**
 * Motor state machine
 **/

const byte M_STOP = 0;
const byte M_FORWARD = 1;
const byte M_REVERSE = 2;
const byte M_LEFT_TURN = 3;
const byte M_RIGHT_TURN = 4;
const byte M_LEFT_PUSH = 5;
const byte M_RIGHT_PUSH = 6;
byte currentMotorState = M_STOP;
unsigned long actionStartTime;
unsigned long actionDuration;

/**
 * Floor photodetector
 **/

float floorAvg;
const float decayRate = 1 - 1 / 20.0;
const float detectEnterThreshold = 0.6;
const float detectExitThreshold = 1.5;
bool enterDetect;
bool exitDetect;

/**
 * IR detector
 **/

const unsigned long beaconDuration =  2000; // microsec
const unsigned long sensorPulse =      400; // microsec ON time
const unsigned long sensorDuration = 15000; // microsec OFF time

byte activeSensor = 0;
byte activeLED = 0;

unsigned long lastBeaconTime = 0;
unsigned long lastSensorTime = 0;

IRLogic sensorState[sensorCount][LEDCount];
boolean sensorDetect[sensorCount];
boolean beaconDetect[sensorCount + 1];

boolean captureDetect = false;
boolean cylinderDetect = false;
boolean leftZone = false;
boolean rightZone = false;

float presence = 0.0;
float presenceDecay = 0.9;
float captureThreshold = 15.0;
float detectThreshold = 3.0;
float leftThreshold = -1.0;
float rightThreshold = 1.0;
float bias = 0.0;
float biasDecay = 0.9;


int dataOut = 0;

/**
 * FSM
 **/

const int leftTurnDuration =   297; // ms ~16th rotation
const int rightTurnDuration =  275; // ms ~16th rotation
const int driveDuration =     2000; // ms ~16 cm forward
const int restDuration =      3000; // ms

/**
 * Servos
 **/

Servo servoLeft;
Servo servoRight;

/**
 * Called once before all the looping starts
 **/
void setup() {

  setupHardwareConnections();

  // Startup indicators
  digitalWrite(redLEDPin, HIGH);
  beep(3);
  delay(1000);
  digitalWrite(redLEDPin, LOW);

  // Initial state
  setMotorState(M_STOP, 2000);
}

/**
 * The actual loop function called by Arduino
 **/
void loop() {

  // Update IR and floor sensors if ready
  if (micros() - lastSensorTime > sensorDuration) {
    updateEyes();
    if (dataOut > 0) {
      debugPrint();
      dataOut--;
    }

    // Only update floor sensor when moving
    if (currentMotorState != M_STOP)
      updateFloorSensor();
  }

  // Update beacon sensor if ready
  if (micros() - lastBeaconTime > beaconDuration)
    updateBeacon();

  // Update FSM if ready
  if (millis() - actionStartTime > actionDuration) {
    // Make next decision
    updateFSM();
  }
}

/**
 * Serial communication for data collection
 **/

void serialEvent() {
  if (Serial.available() && Serial.read() == 120)
    if (dataOut == 0)
      dataOut = 100;
}


// *******************************************************************

/**
 * Sets up hardware connections, such as pins and sensors
 **/

void setupHardwareConnections() {

  // Inputs
  for (byte sensorIndex = 0; sensorIndex < sensorCount; sensorIndex++)
    pinMode(sensorPin[sensorIndex], INPUT);

  pinMode(rearSensorPin, INPUT);

  // Outputs
  for (byte LEDIndex = 0; LEDIndex < LEDCount; LEDIndex++)
    pinMode(LEDPin[LEDIndex], OUTPUT);

  pinMode(redLEDPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);

  // Servo initialization
  servoLeft.attach(motorLPin);
  servoRight.attach(motorRPin);

  // Serial connection
  Serial.begin(115200);

}

void errorSignal() {
  Serial.println();
  Serial.println("OHSHIT!");
  Serial.println();
}

void debugPrint() {
  Serial.print(sensorState[0][0].state * 10);
  Serial.print(',');
  Serial.print(sensorState[1][0].state * 10);
  Serial.print(',');
  Serial.print(sensorState[2][0].state * 10);
  Serial.print(',');
  Serial.print(sensorState[0][1].state * 10);
  Serial.print(',');
  Serial.print(sensorState[1][1].state * 10);
  Serial.print(',');
  Serial.println(sensorState[2][1].state * 10);
}

void setMotorState(byte newMotorState, unsigned long duration) {

  actionStartTime = millis();
  actionDuration = duration;
  currentMotorState = newMotorState;

  if (newMotorState == M_STOP) {
    moveRobot(0);
  }
  else if (newMotorState == M_FORWARD) {
    moveRobot(100);
  }
  else if (newMotorState == M_REVERSE) {
    moveRobot(-100);
  }
  else if (newMotorState == M_LEFT_TURN) {
    turnRobot(0, 100);
  }
  else if (newMotorState == M_RIGHT_TURN) {
    turnRobot(100, 0);
  }
  else if (newMotorState == M_LEFT_PUSH) {
    turnRobot(15, 100);
  }
  else if (newMotorState == M_RIGHT_PUSH) {
    turnRobot(100, 15);
  }
  else errorSignal();
}

void updateBeacon() {
  // Beacon detection MUST be cleared after use for decision-making!
  for (byte i = 0; i < sensorCount; i++)
    beaconDetect[i] |= !digitalRead(sensorPin[i]);
  beaconDetect[sensorCount] = !digitalRead(rearSensorPin);
}

void updateFSM() {
  if (enterDetect) {
    insideSafeZone = true;
    enterDetect = false; // reset flag
  }
  if (exitDetect) {
    insideSafeZone = false;
    exitDetect = false; // reset flag
  }


  if (currentState == S_SEARCH_ARENA) {
    // check if we need to go to another state
    if (insideSafeZone) {
      currentState = S_EXIT_SAFEZONE;
      actionCounter = 0; // reset counter to start from first action
    }
    // if we see a cylinder, or are too close to a wall, enter the corresponding states

    // otherwise, search
    setMotorState(M_FORWARD, 2000);

  } else if (currentState == S_EXIT_SAFEZONE) {
    int actionSequence[][] = {
      {M_REVERSE, 4000},
      {M_RIGHT, rightTurnDuration*5} // May want to turn left sometimes???
    };

    // this could be done much more nicely using object-oriented design...
    if (actionCounter >= actionSequence.length) {
      // action sequence finished, exit state
      currentState = S_SEARCH_ARENA;
      setMotorState(M_STOP, 100);
    } else {
      // otherwise, execute next action
      setMotorState(actionSequence[actionCounter][1], actionSequence[actionCounter][2]);
      actionCounter++;
    }
    // basically want a script here, a sequence of actions
    // [M_REVERSE, 4000]
    // [M_RIGHT/LEFT_TURN, ??]
    // add a list lib (or implement using arrays)

  } else if (currentState == S_SEARCH_SAFEZONE) {
    
    // *** pseudocode ***
    /*
    // situation where at least middle front sensor detects
    if(sensorFront){
      currentState=S_MOVE_TO_SAFEZONE;
    } else if(onlySensorBack){//pushturn 180°
      setMotorstate(6,righTurnDuration*8); 
    } else if (mostlySensorLeft){
      setMotorState(5,leftTurnDuration);
    } else if (mostlySensorRight){
      setMotorState(6,rightTurnDuration);
    } else {//no detection or unclear position
      setMotorState(6,righTurnDuration*4); //pushturn 90°
      setMotorState(1,driveDuration/2); //small step
    }
    */

  } else if (currentState == S_MOVE_TO_SAFEZONE) {

    // *** pseudocode ***
    /*
    //move forward
    setMotorstate(1,driveDuration);  
    //test if signal is lost
    if(!sensorFront) {
      currentState=S_SEARCH_SAFEZONE_HEADING;
    }  
    //if(floorSensor){
    //  currentState=S_EXIT_SAFEZONE;  
    //}
    */

  } else { // this shouldn't happen
    setMotorState(M_STOP, 10000);
    errorSignal();
  }


  digitalWrite(redLEDPin, !digitalRead(redLEDPin));
}

void updateEyes() {

  unsigned int freq;

  freq = sensorState[activeSensor][activeLED].frequency;
  tone(LEDPin[activeLED], freq);
  delayMicroseconds(sensorPulse);

  for (byte i = 0; i < sensorCount; i++) {
    // Read twice for consistency
    sensorDetect[i] = !digitalRead(sensorPin[i]);
    sensorDetect[i] |= !digitalRead(sensorPin[i]);
  }
  noTone(LEDPin[activeLED]);
  lastSensorTime = micros();

  // Avoid reading beacon too soon after IR pulse
  lastBeaconTime = lastSensorTime;

  for (byte i = 0; i < sensorCount; i++)
    sensorState[i][activeLED].mark(freq, sensorDetect[i]);

  if (++activeSensor == sensorCount) activeSensor = 0;
  activeLED = 1 - activeLED;
}

// Floor sensor
void updateFloorSensor() {

  // Safe zone detection MUST be cleared after use for decision-making!
  float floorEye = float(analogRead(floorSensorPin));
  floorAvg = floorAvg * decayRate + floorEye * (1 - decayRate);
  enterDetect |= floorAvg * detectEnterThreshold > floorEye;
  exitDetect |= floorAvg * detectExitThreshold < floorEye;

  //  Serial.print("Current: ");
  //  Serial.println(floorEye);
  //  Serial.print("Average: ");
  //  Serial.println(floorAvg);
  //  Serial.print("Detect: ");
  //  Serial.println(floorDetect);
}

void analyzeIRSensors() {
  
  float p, b;

  b = 2 * sensorState[0][0].state + sensorState[1][0].state
    + sensorState[2][0].state - sensorState[1][1].state
    - 3 * sensorState[2][1].state;
  bias = biasDecay * bias + (1 - biasDecay) * b;

  // Nearer cylinder leads to wider variation in readings
  presence = presenceDecay * presence;
  p = sensorState[0][0].state - sensorState[0][1].state
    + sensorState[1][0].state - sensorState[1][1].state
    + sensorState[2][0].state - sensorState[2][1].state;
  presence += (1 - presenceDecay) * p * p;
  p = sensorState[0][0].state + sensorState[0][1].state
    - sensorState[2][0].state - sensorState[2][1].state;
  presence += (1 - presenceDecay) * p * p;
  p = sensorState[1][0].state + sensorState[1][1].state
    - sensorState[2][0].state - sensorState[2][1].state;
  presence += (1 - presenceDecay) * p * p;
   
  captureDetect |= presence > captureThreshold;
  cylinderDetect |= presence > detectThreshold;
  leftZone |= bias < leftThreshold;
  rightZone |= bias > rightThreshold;

}


// *********************************************************

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

void beep(byte count) {
  while (--count > 0) {
    tone(speakerPin, 2000, 50);
    delay(150);
  }
  tone(speakerPin, 2000, 50);
}
