#include <Servo.h>

// Speed of sound = 331.5 + (0.6 * Temperature in oC)
// distance in cm = echo width in us/2*time for sound to travel 1 cm in us

#define LM35 A0 //Pin A0
#define ECHO 13 // Pin D13
#define TRIG 12 // Pin D12
#define SERVO 10 //Pin D9
#define VCC 11 //vcc pin for URF




// arduino pins
#define COMMON_INPUT_PIN 9
#define LATCH_PIN 2
#define CLOCK_PIN 3
#define DATA_PIN 8



Servo myservo;
float rightDistance;
float leftDistance;

void setup() {
  Serial.begin(9600);
  delay(1000);



  //use pin 11 as a powersource
  pinMode(VCC, OUTPUT);
  digitalShiftWrite(VCC, HIGH);

  pinMode(LM35, INPUT); // Set LM35 to input
  pinMode(ECHO, INPUT); // Set ECHO to input
  pinMode(TRIG, OUTPUT); // Set TRIG to output
  pinMode(SERVO, OUTPUT); // Set SERVO to output

  //attach servo to pin D9 and st its start postion to 0
  myservo.attach(SERVO);
  myservo.write(0);
  delay(500);
}

void loop() {
  //   scan();
  readBothSides();
}

float readDistance() {

  //calculate temp from LM35 reading
  float temp = analogRead(LM35) * 500.0 / 1024; //read LM35 and use constant from https://create.arduino.cc/projecthub/TheGadgetBoy/making-lcd-thermometer-with-arduino-and-lm35-36-c058f0

  //determine constants for calculating distance
  float speedOfSound = 331.5 + (0.6 * temp);
  float usPerMeter = 10000 / speedOfSound;

  // Send >10us pulse to TRIG to enable URF
  digitalShiftWrite(TRIG, HIGH);
  delayMicroseconds(12);
  digitalShiftWrite(TRIG, LOW);

  //calculate distance from pulse width in us returned by pulseIn()
  unsigned long pulseWidth = pulseIn(ECHO, HIGH);
  float distance = pulseWidth / (2 * usPerMeter);

  //if distance calculated to be out of range, set distance to 501
  if (distance >= 501)
    distance = 501;

  return distance;

}


/**
   @param:
          direction, 1 to make servo motor rotate clockwise, 0 to rotate counterclockwise
          it will rotate clockwise if it is other values

*/
void rotate90Deg(int direction) {

  int des = (direction == 0) ? 180 : 0;
  int pos = 90;
  int rotSteps = (direction == 0) ? 10 : -10;

  while (pos != des) {
    myservo.write(pos);
    pos += rotSteps;
    delay(200);
  }
}


void readBothSides() {


  myservo.write(90);
  delay(750);
  
  rotate90Deg(0);
  rightDistance = readDistance();
  Serial.print(rightDistance);
  Serial.print("\n");
  delay(1000);


  
  rotate90Deg(1);
  leftDistance = readDistance();
  Serial.print(leftDistance);
  Serial.print("\n");
  delay(1000);

  myservo.write(90);
  delay(750);

}


void scan() {

  //rotate from 0-180 degrees taking readings intermittently
  int rotSteps = 10;
  int pos;

  for (pos = 0; pos <= 180; pos += rotSteps) {
    myservo.write(pos);
    delay(350);
    readDistance();
  }

  //rotate from 180-0 degrees taking readings intermittently
  for (pos = 180; pos >= 0; pos -= rotSteps) {
    myservo.write(pos);
    delay(350);
    readDistance();
  }
}





// shift register library -------------------------





// display a number on the digital segment display
void digitalShiftWrite(int pin, int value)
{

     // it would be something like 00010000 or 11101111
    int bits = (value == HIGH) ? (int) pow(2.0, pin) : ~ (int) pow(2.0, pin);
    
    // update bit pattern
     // OR makes that bit 1 and keeps other bits; AND makes that bit 0 and keeps other bits
    bitPattern = (value == HIGH) ? (bits | bitPattern) : (bits & bitPattern);
    // set the LATCH_PIN to low potential, before sending data
    digitalWrite(LATCH_PIN, LOW);

    // the original data (bit pattern)
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitPattern);

    // set the LATCH_PIN to high potential, after sending data
    digitalWrite(LATCH_PIN, HIGH);

}

// display a number on the digital segment display
int digitalShiftRead(int pin)
{
    int bits = pow(2.0, pin);
    // set the LATCH_PIN to low potential, before sending data
    digitalWrite(LATCH_PIN, LOW);

    // the original data (bit pattern)
    // assuming that this bit is the only bit set to 1 among all bits used as inputs
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bits | bitPattern);

    // set the LATCH_PIN to high potential, after sending data
    digitalWrite(LATCH_PIN, HIGH);

    delay(2); // delay for debouncing
    int result = digitalRead(COMMON_INPUT_PIN);

    // restore the old bit pattern (dont really need to do this, cuz next time we other bits used as inputs will be set to zero)
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitPattern);
    digitalWrite(LATCH_PIN, HIGH);
    delay(2);

    return result;
}