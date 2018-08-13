
 
// connect to the ST_CP of 74HC595 (pin 3,latch pin)
int latchPin = 2;
// connect to the SH_CP of 74HC595 (pin 4, clock pin)
int clockPin = 3;
// connect to the DS of 74HC595 (pin 2)
int dataPin = 8;

int shiftCommonInputPin = 9;
int transistorCtrlPin1 = 7;
int transistorCtrlPin2 = 6;
 
void setup() {
  // Set latchPin, clockPin, dataPin as output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
//  pinMode(transistorCtrlPin1, OUTPUT);
//  pinMode(transistorCtrlPin2, OUTPUT);
  pinMode(shiftCommonInputPin, INPUT);
  
  digitalWrite(transistorCtrlPin1, LOW);
  digitalWrite(transistorCtrlPin2, HIGH);
}

int dataShiftOut = 0;

// display a number on the digital segment display
void shiftWrite(int digit) {
  dataShiftOut = digit;
  // set the latchPin to low potential, before sending data
  digitalWrite(latchPin, LOW);
     
  // the original data (bit pattern)
  shiftOut(dataPin, clockPin, MSBFIRST, digit);  
 
  // set the latchPin to high potential, after sending data
  digitalWrite(latchPin, HIGH);
}

// display a number on the digital segment display
int shiftRead(int pin) {
  
  // set the latchPin to low potential, before sending data
  digitalWrite(latchPin, LOW);
     
  // the original data (bit pattern)
  shiftOut(dataPin, clockPin, MSBFIRST, dataShiftOut + pin);  
 
  // set the latchPin to high potential, after sending data
  digitalWrite(latchPin, HIGH);

  delay(2);
  return digitalRead(shiftCommonInputPin);
}
 
void loop() {
   for(int pin = 1 ; pin<=8 ; pin*=2)
   {     
    if(shiftRead(pin<<4))
      shiftWrite(pin);
    else
      shiftWrite( dataShiftOut & !pin);
   }

}
