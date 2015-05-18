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

// states for FSM
// const byte S_SEARCH_ARENA = 1;
// const byte S_EXIT_SAFEZONE = 2;
// const byte S_AVOID_WALL = 3;
// const byte S_GRAB_CYLINDER = 4;
// const byte S_INVESTIGATE_OBJECT = 5;
// const byte S_SEARCH_SAFEZONE_HEADING = 6;
// const byte S_MOVE_TO_SAFEZONE = 7;
// byte currentState = 1;


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
    digitalWrite(redLEDPin, !digitalRead(redLEDPin));
    setMotorState(M_STOP, 2000);
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
