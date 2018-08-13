/*************************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE ULTRASONIC SENSOR
 *************************************************************************************************************************************************************************************************/

/**
   @param:
          direction, 1 to make servo motor rotate clockwise, 0 to rotate counterclockwise
          it will rotate clockwise if it is other values

*/
void rotate90Deg(int directionM)
{
  int des = (directionM == 0) ? 180 : 0;
  int pos = 90;
  int rotSteps = (directionM == 0) ? 10 : -10;

  while (pos != des)
  {
    myservo.write(pos);
    pos += rotSteps;
    delay(200);
  }
}

void readBothSides()
{

  myservo.write(90);
  delay(750);

  rotate90Deg(0);
  leftDistance = readDistance();
  Serial.println("Left Distance: "+(String)leftDistance);
  delay(1000);

  rotate90Deg(1);
  rightDistance = readDistance();
  Serial.println("Right Distance: "+(String)rightDistance);
  delay(1000);

  myservo.write(90);
  delay(750);
}

float readDistance()
{

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
