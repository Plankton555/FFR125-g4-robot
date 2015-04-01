#include <Servo.h>

int forwardSpeed = 20; // interval: [-100, 100]
motorLPin = 11;
motorRPin = 12;

// FSM: 0 = search for mines, 1 = avoid obstacle, 2 = mine detected???
int currentState = 0;

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
  bool obstacleDetected = detectObstacle();
  bool mineDetected = detectMine();

  // which state should we be in?
  if (obstacleDetected {
    currentState = 1; // avoid the obstacle
  } else if (mineDetected) {
    currentState = 2; // do something with mine
  } else {
    currentState = 0; // search arena
  }

  // Finite state machine starts here
  if (currentState == 0) {
    ServoL.writeMicroseconds(convertSpeed(forwardSpeed));
    ServoR.writeMicroseconds(convertSpeed(forwardSpeed));
  } else if (currentState == 1) {
    // bla bla
  } else if (currentState == 2) {
    // blabla
  }


  delay(100);
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