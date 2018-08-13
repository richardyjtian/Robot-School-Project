#include <Servo.h>

// Speed of sound = 331.5 + (0.6 * Temperature in oC)
// distance in cm = echo width in us/2*time for sound to travel 1 cm in us

#define LM35 A0 //Pin A0
#define ECHO 13 // Pin D13
#define TRIG 12 // Pin D12
#define SERVO 9 //Pin D9
#define VCC 11 //vcc pin for URF




Servo myservo;
float rightDistance;
float leftDistance;

void setup() {
  Serial.begin(9600);
  delay(1000);



  //use pin 11 as a powersource
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

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
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG, LOW);

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




