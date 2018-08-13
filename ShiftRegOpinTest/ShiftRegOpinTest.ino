
 
// connect to the ST_CP of 74HC595 (pin 3,latch pin)
int latchPin = 2;
// connect to the SH_CP of 74HC595 (pin 4, clock pin)
int clockPin = 3;
// connect to the DS of 74HC595 (pin 2)
int dataPin = 8;
 
void setup() {
  // Set latchPin, clockPin, dataPin as output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

}
 
// display a number on the digital segment display
void sevenSegWrite(int digit) {
  // set the latchPin to low potential, before sending data
  digitalWrite(latchPin, LOW);
     
  // the original data (bit pattern)
  shiftOut(dataPin, clockPin, MSBFIRST, digit);  
 
  // set the latchPin to high potential, after sending data
  digitalWrite(latchPin, HIGH);
}
 
void loop() {       
  for(int i=1 ; i<=15 ; i++)
  {
    sevenSegWrite(i);
    delay(1000);
  }
}
