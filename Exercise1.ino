int distance;
long frequency;

void setup() // Built-in initialization block
{
  tone(4, 3000, 1000); // Play tone for 1 second
  delay(1000); // Delay to finish tone
  pinMode(10, INPUT); 
  pinMode(9, OUTPUT); // Left IR LED & Receiver
  Serial.begin(9600); // Set data rate to 9600 bps
}

void loop() // Main loop auto-repeats
{
  distance=0;
  for(frequency=38000; frequency<42000;frequency+=100){
    int irLeft = irDetect(9, 10, frequency); // Check for object
    distance+=irLeft;
  }
  Serial.println(distance); 
  delay(100);
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