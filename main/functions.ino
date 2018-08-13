void principal_function1(){
  //Measure Distance 
  top:
  currentDistance = readDistance();
  Serial.println("Distance is ");
  Serial.println(currentDistance);
  
  /*if(currentDistance <= SLOW_DOWN){
    slowDown();
    turn(check_l_or_r());
    goto top;
  }*/

  //set motor speed to max

  //setMotorSpeed(LEFT_MOTOR, 21304);
  //setMotorSpeed(RIGHT_MOTOR, 34343);
  //Serial.println("HEEHAW");
  keepStraight(255);
}

int const TAPE = 0, FLOOR = 1;
int const LEFT_OPTIC_BLACK_THRESH = 800,  RIGHT_OPTIC_BLACK_THRESH = 300;
int const LEFT_OPTIC_SENSOR = 1,  RIGHT_OPTIC_SENSOR = 2;

  int stop_counter = 0;
  int line_speed = 80;
void principal_function2(){

  int left_optic_read = (analogRead(LEFT_OPTIC_SENSOR) > LEFT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  int right_optic_read = (analogRead(RIGHT_OPTIC_SENSOR) > RIGHT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  if(left_optic_read == TAPE && right_optic_read == TAPE){
    clearDisplay();
    writeLCDString("Straight");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin,line_speed);
    analogWrite(Motors[RIGHT_MOTOR].enPin,line_speed);
    delay(50);
  }
  else if(left_optic_read == FLOOR && right_optic_read == TAPE){
    clearDisplay();
    writeLCDString("RIGHT");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin,line_speed);
    analogWrite(Motors[RIGHT_MOTOR].enPin,0);
    delay(50);
  }
  else if(left_optic_read == TAPE && right_optic_read == FLOOR){
    clearDisplay();
    writeLCDString("LEFT");
    stop_counter = 0;
    analogWrite(Motors[RIGHT_MOTOR].enPin,line_speed);
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    delay(50);
  }
  else{
    stop_counter++;
    if(stop_counter > 80){
      clearDisplay();
      writeLCDString("STOP");
      analogWrite(Motors[RIGHT_MOTOR].enPin,0);
      analogWrite(Motors[LEFT_MOTOR].enPin,0);
    }
    delay(10);
  }
}

void additional_function(){
  drawWord("E");
}


// buzzes the said pin for the given period in miliseconds
void buzz(int pin, int miliseconds)
{
  for(int i=0; i < miliseconds; i+=2)
  {
    digitalShiftWrite(pin, LOW);
    delay(1);
    digitalShiftWrite(pin, HIGH);
    delay(1);
  }
  digitalShiftWrite(pin, LOW);
}

