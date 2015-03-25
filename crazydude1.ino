
#include <Servo.h>

Servo ServoL;
Servo ServoR;
double threshold;

void setup() // Built-in initialization block
{
  Serial.begin(9600);
  ServoL.attach(13);
  ServoR.attach(12);
  threshold =volts(A3);
  
}
void loop() // Main loop auto-repeats
{
  Serial.print("A3 = "); // Display "A3 = "
  Serial.print(volts(A3)); // Display measured A3 volts
  Serial.println(" volts"); // Display " volts" & newline
  
    if (volts(A3) <= threshold/2.0){
    ServoL.detach();
    ServoR.detach();
  }
  else{
    ServoL.writeMicroseconds(1600);
    ServoR.writeMicroseconds(1400);
    
  }
  delay(100); 

}

float volts(int adPin) // Measures volts at adPin
{ // Returns floating point voltage
  return float(analogRead(adPin)) * 5.0 / 1024.0;
}
