#include <Servo.h>
#include <math.h>

// shift registers pins
#define REG_INPUT_LEVER1 11 // front lever
#define REG_INPUT_LEVER2 10 // back lever

#define SW1 12
#define SW2 13
#define SW3 14
#define SW4 15

#define GREEN_LED 8
#define RED_LED 9

#define BUZZER 0

#define RS 3
#define E 2
#define D4 4
#define D5 5
#define D6 6
#define D7 7

/**********************************************
   MASTER PIN ASSIGNMENTS
 **********************************************/

#define LEFT_HALL_EFFECT 18 //a4
#define RIGHT_HALL_EFFECT 17 //a3
#define LM35 A5
#define COMMON_INPUT_PIN 2
#define TRIG 9
#define ECHO 3
#define CLOCK_PIN 8
#define SERVO 10
#define SERVO_PEN 11
#define LATCH_PIN 12
#define DATA_PIN 13

/**********************************************
   Variables to determine the functionality of the robot
 **********************************************/
unsigned char mode; // 1 for pf1 - 2 for pf2 - 3 for af
unsigned char mask = 0;
#define PF1 1
#define PF2 2
#define AF 3

/**********************************************
   GLOBAL VARIABLES FOR THE ULTRASONIC SENSOR AND SERVO
 **********************************************/
Servo myservo;
float rightDistance;
float leftDistance;
float currentDistance;
unsigned long interval = 0;
bool not_drawn = true;
Servo pen_servo;

/**********************************************
   CONSTANTS FOR THE MOTOR SHIELD
 **********************************************/
const float radius = 3.25 / 2; // assuming the radius is the same for two wheels
const int numOfMotors = 2;

// distance of an object in front of the robot that indicates
// when the robot should stop and slow down
const int TOO_CLOSE = 30;
const int SLOW_DOWN = 50; // 50cm + circumference
//individual motors of the motor shield
// using struct to store infor about two motors and control them
// get help from: https://www.dfrobot.com/wiki/index.php/2A_Motor_Shield_For_Arduino_Twin)_(SKU:DRI0017)

boolean flag = true;

typedef struct
{
  const int enPin;
  const int directionPin;
  const int sensorPin;

  float RPM;
  float Speed;

  const int timeout; //  in millis

  int prev_Reading;
  int rotationCounter;
  unsigned long prev_Millis;
  const unsigned long debounce_Millis;
} MotorContrl; // define this struct as a new type called MotorControl

int E1Pin = 5;
int M1Pin = 4;
int E2Pin = 6;
int M2Pin = 7;

// initialize two motors
// E1 and M1 are right wheel, E2 and M2 are left wheel

MotorContrl Motors[] = {
  {E1Pin, M1Pin, RIGHT_HALL_EFFECT, 0, 0, 5000, 1, 0, 0, 30},
  {E2Pin, M2Pin, LEFT_HALL_EFFECT, 0, 0, 5000, 1, 0, 0, 30},
}; // create an array of MotorControl objects and initialize them 0

const int LEFT_FORWARD = LOW;
const int LEFT_BACKWARD = HIGH;
const int RIGHT_FORWARD = HIGH;
const int RIGHT_BACKWARD = LOW;
const unsigned char RIGHT_MOTOR = 0;
const unsigned char LEFT_MOTOR = 1;

// unsigned long bitPattern = 1<<17;
unsigned long bitPattern = 0x0c00; //pins 11&10 are the lever sensor and are interupt inabled
unsigned char numOfBits = 16;

unsigned char sw1;
unsigned char sw2;
unsigned char sw3;
unsigned char sw4;
unsigned char interrupt = 0;

int font_size = 2;
int space = 1;
bool pen_up;
int prev_length;

void setup()
{
  Serial.begin(9600);

  //setup the servo motor, lm35, and the ultrasonic sensor

  pinMode(LM35, INPUT);   // Set LM35 to input
  pinMode(ECHO, INPUT);   // Set ECHO to input
  pinMode(TRIG, OUTPUT);  // Set TRIG to output
  pinMode(SERVO, OUTPUT); // Set SERVO to output
  pinMode(SERVO_PEN, OUTPUT); // Set SERVO_PEN to output

  pinMode(RIGHT_HALL_EFFECT, INPUT);
  pinMode(LEFT_HALL_EFFECT, INPUT);

  myservo.attach(SERVO);
  myservo.write(80); //needs to be called twice
  delay(330);
  myservo.write(90);
  delay(330);
  pen_servo.attach(SERVO_PEN);
  pen_servo.write(0);
  delay(330);
  pinMode(M1Pin, OUTPUT);
  pinMode(M2Pin, OUTPUT);

  //LCD & Shift Register initialization

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(COMMON_INPUT_PIN, INPUT);

  sw1 = digitalShiftRead(SW1);
  sw2 = digitalShiftRead(SW2);
  sw3 = digitalShiftRead(SW3);
  sw4 = digitalShiftRead(SW4);

  clearBits();
  if (sw4) // only attach interrupt if sw4 is on
    attachInterrupt(digitalPinToInterrupt(COMMON_INPUT_PIN), doISR, RISING);
  mode = setMode();

  // setup lcd and display relevant info
  setupLCD();
  blinkCurser(false);
  setCurserLocation(0, 0);
  writeLCDString("CPEN 291");
  setCurserLocation(1, 0);
  writeLCDString("Team G19");
  delay(1500);
  clearDisplay();
  setCurserLocation(0, 0);
  writeLCDString("Align Wheels");
  setCurserLocation(1, 0);
  writeLCDString("QB! GO! GO!");
  align();
  clearDisplay();
  setCurserLocation(0, 0);
  writeLCDString("Alignment Done!");
   for (int i = 5; i > 0; i--) {
    setCurserLocation(1, 0);
    String s = "Start in " + String(i) + " secs";
    writeLCDString(s);
    writeLCD(i); delay(1000);
  }
}

void loop()
{
  if (interrupt)
  {
    mask = 0;
    clearDisplay();
    setCurserLocation(0, 0);
    writeLCDString("Lever Pushed!");
    digitalShiftWrite(RED_LED, LOW);
    digitalShiftWrite(GREEN_LED, HIGH);
    if (interrupt == 1) //front
    {
      setCurserLocation(1, 0);
      writeLCDString("Move Back!");
      for (int i = 0; i < 4; i++)
      {
        keepAligned(170, RIGHT_BACKWARD, LEFT_BACKWARD);
        buzz(BUZZER, 120);
      }
      keepAligned(170, RIGHT_FORWARD, LEFT_BACKWARD);

    }
    else //back
    {
      setCurserLocation(1, 0);
      writeLCDString("Move Forward!");
      for (int i = 0; i < 4; i++)
      {
        keepAligned(170, RIGHT_FORWARD, LEFT_FORWARD);
        buzz(BUZZER, 120);
      }
      keepAligned(170, RIGHT_FORWARD, LEFT_BACKWARD);
    }
    digitalShiftWrite(GREEN_LED, LOW);
    interrupt = 0;
  }
  else if (mode == PF1 && !interrupt)
  {
    if (!mask)
    {
      clearDisplay();
      setCurserLocation(0, 0);
      writeLCDString("Function 1");
      mask = 1;
    }
    principal_function1();
  }
  else if (mode == PF2 && !interrupt)
  {
    if (!mask)
    {
      clearDisplay();
      setCurserLocation(0, 0);
      writeLCDString("Function 2");
      mask = 1;
    }
    principal_function2();
  }
  else if (mode == AF && !interrupt && not_drawn)
  {
    if (!mask)
    {
      clearDisplay();
      setCurserLocation(1, 0);
      writeLCDString("Additional Function");
      mask = 1;
    }
    drawWord("L");
    drawWord("I");
    drawWord("T");
    not_drawn = false;
  }

  else if (!mask && !interrupt)
  {
    clearDisplay();
    setCurserLocation(0, 0);
    writeLCDString("ERROR: Set Mode");
    setCurserLocation(1, 0);
    writeLCDString("Switch modes & Restart");
    delay(1500);
    mask = 1;
  }
}

/**************************************************************************************************************************************************************************************
   FUNCTIONS TO CHANGE MODE AND ISR
 **************************************************************************************************************************************************************************************/
unsigned char setMode()
{
  if (sw1)
    return PF1;
  if (sw2)
    return PF2;
  if (sw3)
    return AF;
  return 0;
}

//interrupt service routine
void doISR()
{
  int lever1 = digitalShiftRead(REG_INPUT_LEVER1);
  int lever2 = digitalShiftRead(REG_INPUT_LEVER2);
  if (lever1)
  {
    interrupt = 1;
    digitalShiftWrite(RED_LED, HIGH);
    digitalShiftWrite(GREEN_LED, LOW);
    keepAligned(170, RIGHT_BACKWARD, LEFT_BACKWARD);

  }
  else if (lever2)
  {
    interrupt = 2;
    digitalShiftWrite(RED_LED, LOW);
    digitalShiftWrite(GREEN_LED, HIGH);
    keepAligned(170, RIGHT_FORWARD, LEFT_FORWARD);
  }
}

/**************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE MOTOR
 **************************************************************************************************************************************************************************************/
//ALIGNS THE TWO WHEELS SO THAT THE HALL EFFECT SENSORS ARE IN THE SAME POSITION
void align()
{
  int lastRh = digitalRead(RIGHT_HALL_EFFECT);
  int lastLh = digitalRead(LEFT_HALL_EFFECT);
  int rh;
  int lh;
  setMotorDirection(RIGHT_MOTOR, RIGHT_FORWARD);
  setMotorDirection(LEFT_MOTOR, LEFT_FORWARD);
  bool falling_left = false;
  bool falling_right = false;

  do
  {
    analogWrite(Motors[RIGHT_MOTOR].enPin, 90);
    analogWrite(Motors[LEFT_MOTOR].enPin, 90);

    rh = digitalRead(RIGHT_HALL_EFFECT);
    lh = digitalRead(LEFT_HALL_EFFECT);

    if ( (rh == LOW && lastRh == HIGH) || falling_right ) {
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      falling_right = true;
    }

    if ( (lh == LOW && lastLh == HIGH) || falling_left) {
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
      falling_left = true;
    }
    lastRh = rh;
    lastLh = lh;
  } while (!falling_left || !falling_right); // at least one of them is not aligned

}/**
   @params:
          radius of the wheel in cm
   @return:
          speed in cm/sec
*/
float rpmToSpeed(float rpm, float r)
{
  return rpm * 2 * M_PI * r / 60;
}

void initializeMotor()
{
  for (int i = 0; i < numOfMotors; i++)
  {
    digitalWrite(Motors[i].enPin, LOW);
    pinMode(Motors[i].directionPin, OUTPUT);
    pinMode(Motors[i].enPin, OUTPUT);
  }
}

void resetRPMAndSpeed(int motorNum)
{
  Motors[motorNum].RPM = 0;
  Motors[motorNum].Speed = 0;
  Motors[motorNum].prev_Reading = 1;
  Motors[motorNum].rotationCounter = 0;
  Motors[motorNum].prev_Millis = 0;
}

/**void
   @params:
          motorNum, a number represents the motor whose speed is going to be set
          0 is the right motor, and 1 is the left one.

          direction, either FORWARD or BACKWARD
*/
void setMotorDirection(int motorNum, int directionM)
{
  digitalWrite(Motors[motorNum].directionPin, directionM);
}

/**
   keeps the wheels aligned takes the direction on l and r motors
*/
unsigned long keepAligned(int PWM, int rd, int ld)
{
  int lastRh = digitalRead(RIGHT_HALL_EFFECT);
  int lastLh = digitalRead(LEFT_HALL_EFFECT);
  int rh;
  int lh;
  setMotorDirection(RIGHT_MOTOR, rd);
  setMotorDirection(LEFT_MOTOR, ld);
  bool falling_left = false;
  bool falling_right = false;

  unsigned long period = millis();
  do
  {
    analogWrite(Motors[RIGHT_MOTOR].enPin, PWM);
    analogWrite(Motors[LEFT_MOTOR].enPin, PWM);

    rh = digitalRead(RIGHT_HALL_EFFECT);
    lh = digitalRead(LEFT_HALL_EFFECT);

    if ( (rh == LOW && lastRh == HIGH) || falling_right ) {
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      falling_right = true;
    }

    if ( (lh == LOW && lastLh == HIGH) || falling_left) {
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
      falling_left = true;
    }
    
    lastRh = rh;
    lastLh = lh;

  } while (!falling_left || !falling_right); // at least one of them is not aligned

  period = millis() - period;
  return period;
}

/**************************************************************************************************************************************************************************************
   PRINCIPAL FUNCTION 1
 **************************************************************************************************************************************************************************************/
void principal_function1()
{
  //Measure Distance
  currentDistance = readDistance(1); // read once 
  clearDisplay();
  setCurserLocation(0, 0);
  writeLCDString(String(currentDistance, 2));
  
  if (currentDistance > SLOW_DOWN)
    interval = keepAligned(255, RIGHT_FORWARD, LEFT_FORWARD);
  else if (currentDistance > TOO_CLOSE)
    interval = keepAligned(currentDistance * 4 + 50, RIGHT_FORWARD, LEFT_FORWARD);
  else
  {
    align();
    readBothSides();

    if (leftDistance < rightDistance)
      keepAligned(160, RIGHT_BACKWARD, LEFT_FORWARD);
      
    else
      keepAligned(160, RIGHT_FORWARD, LEFT_BACKWARD);
    align();
  }
  setCurserLocation(1, 0);
  writeLCDString(String((60000.0 / (2 * interval)), 2));
}

/**************************************************************************************************************************************************************************************
   PRINCIPAL FUNCTION 2
 **************************************************************************************************************************************************************************************/
int const TAPE = 0, FLOOR = 1;
int LEFT_OPTIC_BLACK_THRESH,  RIGHT_OPTIC_BLACK_THRESH;
boolean first = true;
int const LEFT_OPTIC_SENSOR = 1,  RIGHT_OPTIC_SENSOR = 2;

int stop_counter = 0;
int line_speed = 100;
  
void principal_function2(){
  if(first == true){
    delay(5000);
    clearDisplay();
    writeLCDString("SCANNING");
    int RIGHT_OPTIC_TAPE = analogRead(RIGHT_OPTIC_SENSOR); //read large value when on tape
    int LEFT_OPTIC_TAPE = analogRead(LEFT_OPTIC_SENSOR);
    analogWrite(Motors[LEFT_MOTOR].enPin, 90);
    delay(2000);
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);

    /*
     * More precise implementation that checks a range of values instead of just on and off the tape
    //turn right until right sensor is on floor but left sensor is on tape
    setMotorDirection(RIGHT_MOTOR, BACKWARD);
    while(fabs(analogRead(RIGHT_OPTIC_SENSOR)-RIGHT_OPTIC_MAX) < 300){
      analogWrite(Motors[RIGHT_MOTOR].enPin,90);
    }
    analogWrite(Motors[RIGHT_MOTOR].enPin,0);
    setMotorDirection(RIGHT_MOTOR, FORWARD);
    int RIGHT_OPTIC_MIN = analogRead(RIGHT_OPTIC_SENSOR);
    if(analogRead(LEFT_OPTIC_SENSOR) < LEFT_OPTIC_MAX)
      LEFT_OPTIC_MAX = analogRead(LEFT_OPTIC_SENSOR);
    RIGHT_OPTIC_BLACK_THRESH = (RIGHT_OPTIC_MAX + RIGHT_OPTIC_MIN)/2;
    while(analogRead(RIGHT_OPTIC_SENSOR) < RIGHT_OPTIC_BLACK_THRESH){
      analogWrite(Motors[RIGHT_MOTOR].enPin,90); //keep turning right until the left sensor is on the tape again
    }
    analogWrite(Motors[RIGHT_MOTOR].enPin,0);
    //turn left until left sensor is on floor but right sensor is on tape
    setMotorDirection(LEFT_MOTOR, BACKWARD);
    while(fabs(analogRead(LEFT_OPTIC_SENSOR)-LEFT_OPTIC_MAX) < 300){
      analogWrite(Motors[LEFT_MOTOR].enPin,90);
    }
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    setMotorDirection(LEFT_MOTOR, FORWARD);
    int LEFT_OPTIC_MIN = analogRead(LEFT_OPTIC_SENSOR);
    if(analogRead(RIGHT_OPTIC_SENSOR) < RIGHT_OPTIC_MAX)
      RIGHT_OPTIC_MAX = analogRead(RIGHT_OPTIC_SENSOR);
    LEFT_OPTIC_BLACK_THRESH = (LEFT_OPTIC_MAX + LEFT_OPTIC_MIN)/2;
    while(analogRead(LEFT_OPTIC_SENSOR) < LEFT_OPTIC_BLACK_THRESH){
      analogWrite(Motors[LEFT_MOTOR].enPin,90);
    }
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    */
    
    int RIGHT_OPTIC_FLOOR = analogRead(RIGHT_OPTIC_SENSOR);
    int LEFT_OPTIC_FLOOR = analogRead(LEFT_OPTIC_SENSOR);
    
    LEFT_OPTIC_BLACK_THRESH = (LEFT_OPTIC_TAPE + LEFT_OPTIC_FLOOR)/2;
    RIGHT_OPTIC_BLACK_THRESH = (RIGHT_OPTIC_TAPE + RIGHT_OPTIC_FLOOR)/2;
    setMotorDirection(LEFT_MOTOR, LEFT_BACKWARD);
    while(analogRead(RIGHT_OPTIC_SENSOR) < RIGHT_OPTIC_BLACK_THRESH || analogRead(LEFT_OPTIC_SENSOR) < LEFT_OPTIC_BLACK_THRESH){ //analogRead(LEFT_OPTIC_SENSOR) < LEFT_OPTIC_BLACK_THRESH
      analogWrite(Motors[LEFT_MOTOR].enPin,90); //keep turning right until the left sensor is on the tape again
    }
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    setMotorDirection(LEFT_MOTOR, LEFT_FORWARD);
    
    first = false;
    clearDisplay();
    setCurserLocation(0, 0);
    writeLCDString("RIGHT T:" + (String)RIGHT_OPTIC_BLACK_THRESH);
    setCurserLocation(1, 0);
    writeLCDString("LEFT T:" + (String)LEFT_OPTIC_BLACK_THRESH);
    delay(5000);
  }
  
  int left_optic_read = (analogRead(LEFT_OPTIC_SENSOR) > LEFT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  int right_optic_read = (analogRead(RIGHT_OPTIC_SENSOR) > RIGHT_OPTIC_BLACK_THRESH) ? TAPE : FLOOR;
  
  if(left_optic_read == TAPE && right_optic_read == TAPE){
    clearDisplay();
    writeLCDString("STRAIGHT");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin,line_speed);
    analogWrite(Motors[RIGHT_MOTOR].enPin,line_speed);
  }
  else if(left_optic_read == FLOOR && right_optic_read == TAPE){
    clearDisplay();
    writeLCDString("RIGHT");
    stop_counter = 0;
    analogWrite(Motors[RIGHT_MOTOR].enPin,0);
    analogWrite(Motors[LEFT_MOTOR].enPin,line_speed*9/10);
  }
  else if(left_optic_read == TAPE && right_optic_read == FLOOR){
    clearDisplay();
    writeLCDString("LEFT");
    stop_counter = 0;
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    analogWrite(Motors[RIGHT_MOTOR].enPin,line_speed*9/10);
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
  clearDisplay();
  setCurserLocation(0, 0);
  writeLCDString(String(leftDistance, 2));
  delay(100);
  rotate90Deg(1);
  delay(800);
  rightDistance = readDistance(20);
  writeLCDString(String(rightDistance, 2));  
  delay(100);
  myservo.write(90);
  delay(800);
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

// read pin from pin needs to be from 12-15 otherwise code will be compromised
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

  // restore the old bit pattern need to do so to keep the interrupt on level sensors
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

    if(numOfMeasurements != 1)
      delay(60); // cycle period recommended by the datasheet
  }

  distance = sum / numOfMeasurements;
  if (distance > 400)
    distance = 400.0;
  else if (distance < 5)
    distance = 5.0;
    
  return distance;
}

/**************************************************************************************************************************************************************************************
   ADDITIONAL FUNCTION
 **************************************************************************************************************************************************************************************/
void drawWord(char *word){
  int i = 0;
  if(strlen(word) > 1){
    for(int i = 0; i < strlen(word); i++){
      drawWord(word[i]);
    }  
    return;
  }
  
  switch(word[i]){

    case 'a':
    case 'A':
//      //int A_hypotenuse = font_size/cos(15);
//      rotate(345);
//      lowerPen();
//      drawLine(A_hypotenuse, true, 1/2);
//      rotate(105);
//      lowerPen();
//      drawLine(A_hypotenuse*cos(75), false, 0.0);
//      rotate(105);
//      lowerPen();
//      drawLine(A_hypotenuse/2, true, 1.0);
//      lowerPen();
//      drawLine(A_hypotenuse/2, false, 0.0);
//      rotate(75);
//      drawLine(space, false, 0.0);
//      rotate(90);
    return;
    
    case 'b':
    case 'B':
    return;
    
    case 'c':
    case 'C':
    return;
    
    case 'd':
    case 'D':
    return;
    
    case 'e':
    case 'E':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/3, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      liftPen();
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'f':
    case 'F':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/3, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(space + font_size/2, false, 0.0);
      rotate(90);
    return;
    
    case 'g':
    case 'G':////////////////////////////////////////////////////////
    return;
    
    case 'h':
    case 'H':
      lowerPen();
      drawLine(font_size, true, 1/2);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;

    case '1':
    case 'i':
    case 'I':
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(-90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'j':
    case 'J':
    return;
    
    case 'k':
    case 'K':
      lowerPen();
      drawLine(font_size, true, 1/2);
      rotate(335);
      lowerPen();
      drawLine(font_size/(2*cos(25)), true, 1.0);
      rotate(50);
      lowerPen();
      drawLine(font_size/(2*cos(25)), false, 0.0);
      rotate(65);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'l':
    case 'L':
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(-90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      liftPen();
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'm':
    case 'M':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'n':
    case 'N':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(165);
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;

    case '0':
    case 'o':
    case 'O':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(-90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(-90);
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(-90);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'p':
    case 'P':
    return;
    
    case 'q':
    case 'Q':
    return;
    
    case 'r':
    case 'R':
    return;
    
    case 's':
    case 'S':
    return;
    
    case 't':
    case 'T':
      drawLine(font_size, false, 0.0);
      rotate(-90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      keepAligned(255, RIGHT_BACKWARD, LEFT_BACKWARD);
      align();
      rotate(-90);
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'u':
    case 'U':////////////////////////////
    return;
    
    case 'v':
    case 'V':
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(195);
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90); 
    return;
    
    case 'w':
    case 'W':
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/(3*cos(15)),false, 0.0);
      rotate(210);
      lowerPen();
      drawLine(font_size/(3*cos(15)),false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
      rotate(195);
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90); 
    return;
    
    case 'x':
    case 'X':
      rotate(340);
      lowerPen();
      drawLine(font_size/cos(20), false, 0.0);
      rotate(110);
      drawLine((cos(70)*font_size)/cos(20), false, 0.0);
      rotate(110);
      lowerPen();
      drawLine(font_size/cos(20), false, 0.0);
      rotate(70);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'y':
    case 'Y':
    return;
    
    case 'z':
    case 'Z':
//      drawLine(font_size, false, 0.0);
//      rotate(270);
//      drawLine,(font_size/2, false, 0.0);
//      rotate(200);
//      drawLine(font_size/(2*cos(20)));
    return;
    
  }
}
  void rotate(int degrees){
    int num = degrees/90;
    if(!pen_up)
      liftPen();
    
    //rotate "degrees" degrees counter-clockwise
    if(num >= 0){
      for(int i = 1; i <= num; i++){
        keepAligned(160, RIGHT_FORWARD, LEFT_BACKWARD);
        align();
      }
    } 
    else {
      num = -num;
      for(int i = 1; i <= num; i++){
        keepAligned(160, RIGHT_BACKWARD, LEFT_FORWARD);
        align();
      }
    }
    delay(500);
  }

  //length is how long of a line to draw
  //backtrack is whether or not the robot will return to a point along the line it just drew
  //fraction is how far along the line the robot just drew to backtrack, 0.0 <= fraction <= 1.0
  void drawLine(int length, bool backtrack, float fraction){
      for(int i = 1; i <= length; i++){
        keepAligned(255, RIGHT_FORWARD, LEFT_FORWARD);  
        align();
      }
    
    prev_length = length;
    delay(500);
    
    if(backtrack){
      if(!pen_up)
        liftPen();
      for(int i = 1; i <= length; i++){
        keepAligned(255, RIGHT_BACKWARD, LEFT_BACKWARD); 
        align(); 
      }
//      rotate(180);
//      drawLine(prev_length*fraction, false, 0.0);
    }
  }
//  
//  void backtrack(float fraction){
//      if(!pen_up)
//        liftPen();
//      rotate(180);
//      drawLine(prev_length*fraction, false, 0.0);
//  }

  void liftPen(){
    pen_servo.write(20);
    pen_up = true;
  }

  void lowerPen(){
    pen_servo.write(0);
    pen_up = false;
  }
  
