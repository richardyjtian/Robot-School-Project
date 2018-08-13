
void principal_function1()
{
  //Measure Distance
  currentDistance = readDistance(1); // read once 
  //Serial.println("Current distance: " + String(currentDistance, 2));
  if (currentDistance > SLOW_DOWN)
    interval = keepAligned(255, RIGHT_FORWARD, LEFT_FORWARD);
  else if (currentDistance > TOO_CLOSE)
    interval = keepAligned(currentDistance * 4 + 50, RIGHT_FORWARD, LEFT_FORWARD);
  else
  {
    //Serial.println("TOO CLOSE!");
    align();
    readBothSides();
    //Serial.println(leftDistance);
    //Serial.println(rightDistance);
    //Serial.println("");
    if (leftDistance < rightDistance)
      keepAligned(160, RIGHT_BACKWARD, LEFT_FORWARD);
      
    else
      keepAligned(160, RIGHT_FORWARD, LEFT_BACKWARD);
    align();
  }
  setCurserLocation(1, 0);
  writeLCDString(String((60000.0 / (2 * interval)), 2));
}

int const TAPE = 0, FLOOR = 1;
int const LEFT_OPTIC_BLACK_THRESH = 800, RIGHT_OPTIC_BLACK_THRESH = 300;
int const LEFT_OPTIC_SENSOR = 1, RIGHT_OPTIC_SENSOR = 2;

int stop_counter = 0;
int line_speed = 80;
void principal_function2()
{

  int left_optic_read = (analogRead(LEFT_OPTIC_SENSOR) > LEFT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  int right_optic_read = (analogRead(RIGHT_OPTIC_SENSOR) > RIGHT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  if (left_optic_read == TAPE && right_optic_read == TAPE)
  {
    clearDisplay();
    writeLCDString("Straight");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin, line_speed);
    analogWrite(Motors[RIGHT_MOTOR].enPin, line_speed);
    delay(50);
  }
  else if (left_optic_read == FLOOR && right_optic_read == TAPE)
  {
    clearDisplay();
    writeLCDString("RIGHT");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin, line_speed);
    analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    delay(50);
  }
  else if (left_optic_read == TAPE && right_optic_read == FLOOR)
  {
    clearDisplay();
    writeLCDString("LEFT");
    stop_counter = 0;
    analogWrite(Motors[RIGHT_MOTOR].enPin, line_speed);
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    delay(50);
  }
  else
  {
    stop_counter++;
    if (stop_counter > 80)
    {
      clearDisplay();
      writeLCDString("STOP");
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    }
    delay(10);
  }
}

//void additional_function(){
//  drawWord("E");
//}

// buzzes the said pin for the given period in miliseconds
void buzz(int pin, int miliseconds)
{
  for (int i = 0; i < miliseconds; i += 2)
  {
    digitalShiftWrite(pin, LOW);
    delay(1);
    digitalShiftWrite(pin, HIGH);
    delay(1);
  }
  digitalShiftWrite(pin, LOW);
}

void readBothSides()
{
  myservo.write(90);
  rotate90Deg(0);
  delay(500);
  leftDistance = readDistance(20);
  delay(500);
  rotate90Deg(1);
  delay(700);
  rightDistance = readDistance(20);
  delay(500);
  myservo.write(90);
  delay(850);
}

void rotate90Deg(int directionM)
{
  int des = (directionM == 0) ? 180 : 0;
  myservo.write(des);
  delay(2000);
}

/*************************************************************************************************************************************************************************************************
   SHIFT REGISTER FUNCTIONS
 *************************************************************************************************************************************************************************************************/
//set all pins on shift register to 0
void clearBits()
{
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
  digitalWrite(LATCH_PIN, HIGH);
}

// read pin from pin needs to be from 12-15 other wise code will be comprimised
int digitalShiftRead(int pin)
{
  unsigned long temp = bitPattern;
  temp = temp & 0x00ff;
  temp = temp | (1 << pin);

  // set the LATCH_PIN to low potential, before sending data
  digitalWrite(LATCH_PIN, LOW);

  // the original data (temp part of bitpattern)
  // assuming that this bit is the only bit set to 1 among all bits used as inputs
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, ((temp & 0xff00) >> 8)); //send upper 8 bits
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (temp & 0x00ff));        //send lower 8 bits

  // set the LATCH_PIN to high potential, after sending data
  digitalWrite(LATCH_PIN, HIGH);

  delay(10);                                  // delay for debouncing and for the shift register to update
  int result = digitalRead(COMMON_INPUT_PIN); //read common input if high the input pin is high

  // restore the old bit pattern need to do so to keep the interuppt on level sensors
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, ((bitPattern & 0xff00) >> 8));
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (bitPattern & 0x00ff));
  digitalWrite(LATCH_PIN, HIGH);

  delay(5); //wait for shift register to update

  return result;
}

// display a number on the digital segment display
void digitalShiftWrite(int pin, int value)
{

  // it would be something like 00010000 or 11101111
  unsigned long bits = (value == HIGH) ? (1 << pin) : ~(1 << pin) | 0xfc00; //long doesnt sign extend

  // update bit pattern
  // OR makes that bit 1 and keeps other bits; AND makes that bit 0 and keeps other bits
  bitPattern = (value == HIGH) ? (bits | bitPattern) : (bits & bitPattern);

  // set the LATCH_PIN to low potential, before sending data
  digitalWrite(LATCH_PIN, LOW);

  // the original data (bit pattern)
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, ((bitPattern & 0xff00) >> 8));
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (bitPattern & 0x00ff));

  // set the LATCH_PIN to high potential, after sending data
  digitalWrite(LATCH_PIN, HIGH);
}

/*************************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE LCD
 *************************************************************************************************************************************************************************************************/

const int LCDPins[] = {D4, D5, D6, D7};

//got help from https://stackoverflow.com/questions/36274902
void displayNumber(int num)
{
  char number[sizeof(int) * 4 + 1]; // plus 1 for \0
  sprintf(number, "%d", num);
  writeLCDString(number);
}

void shiftCurser(bool right)
{                             //true to right shift false for left shift
  digitalShiftWrite(RS, LOW); // instructions
  if (right)
  {
    writeInstruction(0x14);
  }
  else
  {
    writeInstruction(0x10);
  }
}

void shiftDisplay(bool right)
{                             //true to right shift false for left shift
  digitalShiftWrite(RS, LOW); // instructions
  if (right)
  {
    writeInstruction(0x1C);
  }
  else
  {
    writeInstruction(0x18);
  }
}

void curserHome()
{                             //Returns the cursor to the home position (Address 0). Returns display to its original state if it was shifted.
  digitalShiftWrite(RS, LOW); // instructions
  writeInstruction(0x02);
  delay(2);
}

void setCurserLocation(int row, int col)
{
  int address = (row == 1) ? 0x40 : 0x00;
  address += col;             //the address value is this
  address += 0x80;            //but D7 needs to be high so 1AAA AAAA
  digitalShiftWrite(RS, LOW); // instructions
  writeInstruction(address);
}

void blinkCurser(bool doesBlink)
{
  digitalShiftWrite(RS, LOW); // instructions
  if (doesBlink)
    writeInstruction(0x0F);
  else
    writeInstruction(0x0E);
}

void writeLCDString(String s)
{
  int i = 0;
  while (s[i] != '\0')
  {
    writeLCD(s[i]);
    i++;
  }
}

void writeLCD(char character)
{
  digitalShiftWrite(RS, HIGH); // there are all data
  int value = (int)character;
  writeInstruction(value);
}

void setupLCD()
{
  digitalShiftWrite(RS, LOW); // there are all instructions

  delay(40); //wait for Vcc to rice to 5v

  writeLCDPins(0x3); //FUNCTION SET 8bits

  delay(5); //delay >4.1ms

  writeLCDPins(0x3);      //FUNCTION SET 8bits
  delayMicroseconds(105); //delay >100us

  writeLCDPins(0x3); //FUNCTION SET 8bits

  writeLCDPins(0x2); //FUNCTION SET 4bits

  writeInstruction(0x28); // N=HIGH as we have 2 lines and F=0 font is 5*8

  writeInstruction(0x08); // DisplayOFF

  writeInstruction(0x01); // DisplayClear
  delay(2);

  writeInstruction(0x06); // Increment counter + no Shift
}

void clearDisplay()
{
  digitalShiftWrite(RS, LOW);
  writeInstruction(0x01);
  delay(2);
}
void writeInstruction(int value)
{
  writeLCDPins((value & 0xF0) >> 4); //upper 4 bits;
  writeLCDPins(value & 0x0F);        //lower 4 bits
  delayMicroseconds(40);
}
void writeLCDPins(int val)
{
  for (int i = 0; i < 4; i++)
    digitalShiftWrite(LCDPins[i], (val & (1 << i)) == (1 << i));
  pulseE();
}

void pulseE(void)
{
  digitalShiftWrite(E, LOW);
  delayMicroseconds(1); //wait 150ns
  digitalShiftWrite(E, HIGH);
  delayMicroseconds(1); //wait 150ns
  digitalShiftWrite(E, LOW);
  delayMicroseconds(10); //wait 150ns
}

float readDistance(int num)
{
  // datasheet: https://www.robotshop.com/media/files/pdf2/hc-sr04-ultrasonic-range-finder-datasheet.pdf
  float speedOfSound;
  int numOfMeasurements = num;
  float usPerMeter;
  float temp;
  float distance;

  float sum = 0;
  for (int i = 0; i < numOfMeasurements; i++)
  {
    //calculate temp from LM35 reading
    temp = analogRead(LM35) * 500.0 / 1024; //read LM35 and use constant from https://create.arduino.cc/projecthub/TheGadgetBoy/making-lcd-thermometer-with-arduino-and-lm35-36-c058f0
                                                  //  float temp = 23;
    //determine constants for calculating distance
    delay(100);
    speedOfSound = 331.5 + (0.6 * temp);
    usPerMeter = 10000.0 / speedOfSound;

    // Send >10us pulse to TRIG to enable URF
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2); // make sure trig is actually low
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(12);
    digitalWrite(TRIG, LOW);

    //calculate distance from pulse width in us returned by pulseIn()
    distance = pulseIn(ECHO, HIGH) / (2 * usPerMeter);

    if (distance > 400)
      distance = 400;
    else if (distance < 5)
      distance = 5;

    sum+= distance;

    delay(60); // cycle period recommended by the datasheet
  }
  
  distance = sum / numOfMeasurements;
  if (distance > 400)
    distance = 400.0;
  else if (distance < 5)
    distance = 5.0;
    
  return distance;
}
