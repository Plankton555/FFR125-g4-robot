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

// Parameters
const unsigned long UPDATE_INTERVAL = 100; // milliseconds
const unsigned long sensorDelay = 300;     // microseconds ON time, full cycle is 3x


// Variables
Servo servoLeft;
Servo servoRight;

IRLogic sensorState[sensorCount][LEDCount];
boolean sensorDetect[sensorCount];

boolean carryingCylinder = false;

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
}

void debugPrint() {
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
  if (carryingCylinder) {
    // Find safe zone and return cylinder to it

  } else {
    // Roam arena and find a cylinder
    performRoamingBehavior();
  }
}

void performRoamingBehavior() {
  double rndNr = ((double)random(0, 100)) / 100;
  double rndChange = (rndNr - 0.5) * 2 * roam_change; // change seed here
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
  return 1500 - s * 2;
}

int convertSpeedL(int s) {
  return 1500 + s * 2;
}

void stopRobot() {
  servoLeft.writeMicroseconds(convertSpeedL(0));
  servoRight.writeMicroseconds(convertSpeedR(0));
}
