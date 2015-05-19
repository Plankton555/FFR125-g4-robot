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

const byte S_SEARCH_ARENA = 100;
const byte S_SEARCH_FORWARD = 101;
const byte S_SEARCH_PAUSE = 102;
const byte S_SEARCH_WALL_DETECTED = 113;
const byte S_SEARCH_EVADE_LEFT = 103;
const byte S_SEARCH_EVADE_RIGHT = 104;
const byte S_SEARCH_EVADE_LEFT_PAUSE = 105;
const byte S_SEARCH_EVADE_RIGHT_PAUSE = 106;
const byte S_SEARCH_LEAVE_BACK = 107;
const byte S_SEARCH_LEAVE_TURN = 108;
const byte S_SEARCH_CAPTURE = 109;
const byte S_SEARCH_FIND_LEFT = 110;
const byte S_SEARCH_FIND_RIGHT = 111;
const byte S_SEARCH_TURN_PAUSE = 112;

const byte S_RETURN_START = 200;
const byte S_RETURN_FORWARD = 201;
const byte S_RETURN_TURN_LEFT = 202;
const byte S_RETURN_TURN_RIGHT = 209;
const byte S_RETURN_WALL_DETECTED = 203;
const byte S_RETURN_EVADE_LEFT = 204;
const byte S_RETURN_EVADE_RIGHT = 205;
const byte S_RETURN_EVADE_LEFT_PAUSE = 206;
const byte S_RETURN_EVADE_RIGHT_PAUSE = 207;
// const byte S_RETURN_FINISHED = 208;

const byte S_EXIT_SAFEZONE = 2;
const byte S_AVOID_WALL = 3;
const byte S_GRAB_CYLINDER = 4;
const byte S_INVESTIGATE_OBJECT = 5;
const byte S_SEARCH_SAFEZONE = 6;
const byte S_MOVE_TO_SAFEZONE = 7;
byte currentState = 1;

int actionCounter = 0;
bool insideSafeZone = false;

// -1: left, +1: right
byte lastReturnMove = 1;
bool lastReturnMoveWasForward = false;


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
boolean wallDetect = false;
boolean leftZone = false;
boolean rightZone = false;

float presence = 0.0;
float presenceDecay = 0.9;
float captureThreshold = 1.5;
float detectThreshold = 0.3;
float wallThreshold = 0.45;
float leftThreshold = -0.1;
float rightThreshold = 0.1;
float bias = 0.0;
float biasDecay = 0.9;


int dataOut = 0;

/**
 * FSM
 **/

const int leftTurnDuration =   297; // ms ~16th rotation
const int rightTurnDuration =  275; // ms ~16th rotation
const int driveDuration =     2000; // ms ~33 cm forward
const int restDuration =      0500; // ms

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

  currentState = S_RETURN_START;//0; //S_SEARCH_ARENA;
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

    // Analze sensor data
    analyzeIRSensors();
    //Serial.println(wallDetect);

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
  bool resetFlags = true;

  if (currentState == S_SEARCH_ARENA) {
    currentState = S_SEARCH_FORWARD;
    // If wall detected
    if (wallDetect) {
      currentState = S_SEARCH_WALL_DETECTED;
    }


  } else if (currentState == S_SEARCH_FORWARD) {
    setMotorState(M_FORWARD, driveDuration);
    currentState = S_SEARCH_PAUSE;
  } else if (currentState == S_SEARCH_PAUSE) {
    setMotorState(M_STOP, restDuration);
    currentState = S_SEARCH_ARENA;
  } else if (currentState == S_SEARCH_WALL_DETECTED) {
    setMotorState(M_REVERSE, driveDuration / 4);
    if (bias > 0) {
      currentState = S_SEARCH_EVADE_LEFT;
    } else {
      currentState = S_SEARCH_EVADE_RIGHT;
    }
  } else if (currentState == S_SEARCH_EVADE_LEFT) {
    setMotorState(M_LEFT_TURN, leftTurnDuration * 1);
    currentState = S_SEARCH_EVADE_LEFT_PAUSE;
  } else if (currentState == S_SEARCH_EVADE_RIGHT) {
    setMotorState(M_RIGHT_TURN, leftTurnDuration * 1);
    currentState = S_SEARCH_EVADE_RIGHT_PAUSE;
  } else if (currentState == S_SEARCH_EVADE_LEFT_PAUSE) {
    setMotorState(M_STOP, restDuration);
    // If wall is still detected
    if (wallDetect) {
      currentState = S_SEARCH_EVADE_LEFT;
    } else {
      currentState = S_SEARCH_ARENA;
    }
  } else if (currentState == S_SEARCH_EVADE_RIGHT_PAUSE) {
    setMotorState(M_STOP, restDuration);
    // If wall is still detected
    if (wallDetect) {
      currentState = S_SEARCH_EVADE_RIGHT;
    } else {
      currentState = S_SEARCH_ARENA;
    }
  } else if (currentState == S_SEARCH_LEAVE_BACK) {
    setMotorState(M_REVERSE, driveDuration);
    currentState = S_SEARCH_LEAVE_TURN;
  } else if (currentState == S_SEARCH_LEAVE_TURN) {
    setMotorState(M_LEFT_TURN, leftTurnDuration * 7);
    currentState = S_SEARCH_PAUSE;
  } else if (currentState == S_SEARCH_CAPTURE) {
  } else if (currentState == S_SEARCH_FIND_LEFT) {
    setMotorState(M_LEFT_TURN, leftTurnDuration);
    currentState = S_SEARCH_TURN_PAUSE;
  } else if (currentState == S_SEARCH_FIND_RIGHT) {
    setMotorState(M_RIGHT_TURN, leftTurnDuration);
    currentState = S_SEARCH_TURN_PAUSE;
  } else if (currentState == S_SEARCH_TURN_PAUSE) {
    setMotorState(M_STOP, restDuration);
    currentState = S_SEARCH_PAUSE;
  }










  // Return states
  else if (currentState == S_RETURN_START) {
    Serial.println("RETURN START");
    // Beacon seen during turn
    if (beaconDetect[0]) {
      currentState = S_RETURN_FORWARD;
    } else {
      if (lastReturnMoveWasForward) {
        if (lastReturnMove == 1) {
          currentState = S_RETURN_TURN_LEFT;
        } else {
          currentState = S_RETURN_TURN_RIGHT;
        }
      } else {
        currentState = S_RETURN_TURN_LEFT;
      }
    }

  } else if (currentState == S_RETURN_FORWARD) {

    Serial.println("RETURN FORWARD");
    setMotorState(M_FORWARD, driveDuration / 2);
    currentState = S_RETURN_START;
    lastReturnMoveWasForward = true;
  } else if (currentState == S_RETURN_TURN_LEFT) {

    Serial.println("RETURN LEFT");
    lastReturnMove = -1;
    setMotorState(M_LEFT_PUSH, leftTurnDuration * 2);
    currentState = S_RETURN_START;
    lastReturnMoveWasForward = false;
  } else if (currentState == S_RETURN_TURN_RIGHT) {

    Serial.println("RETURN RIGHT");
    lastReturnMove = 1;
    setMotorState(M_RIGHT_PUSH, leftTurnDuration * 2);
    currentState = S_RETURN_START;
    lastReturnMoveWasForward = false;
  }
  // } else if (currentState == S_RETURN_WALL_DETECTED) {
  // } else if (currentState == S_RETURN_EVADE_LEFT) {
  // } else if (currentState == S_RETURN_EVADE_RIGHT) {
  // } else if (currentState == S_RETURN_EVADE_LEFT_PAUSE) {
  // } else if (currentState == S_RETURN_EVADE_RIGHT_PAUSE) {
  // }
  else {
    // DEBUG STATE
    setMotorState(M_STOP, 200);


    Serial.println(beaconDetect[1]);
    Serial.println(beaconDetect[3]);
    Serial.println();
  }

  // Reset flags
  if (true) {
    wallDetect = false;
    beaconDetect[0] = false;
    beaconDetect[1] = false;
    beaconDetect[2] = false;
    beaconDetect[3] = false;
  }

  // State change indicator
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

  // Wall detection
  float f1, f2, fAvg;
  f1 = sensorState[1][0].state;
  f2 = sensorState[1][1].state;
  fAvg = (f1 + f2 + min(f1, f2)) / 3.0;

  // Serial.println(fAvg);
  // get position of wall;
  wallDetect |= fAvg < wallThreshold;
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
