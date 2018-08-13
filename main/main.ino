#include <Servo.h>
#include <math.h>

// shift registers pins
#define REG_INPUT_LEVER1 10
#define REG_INPUT_LEVER2 11

#define SW1 12
#define SW2 13
#define SW3 14
#define SW4 15

#define GREEN_LED 8
#define RED_LED 9

#define BUZZER 0
#define TRIG 7

#define RS 1
#define E 2
#define D4 3
#define D5 4
#define D6 5
#define D7 6


/**********************************************
   MASTER PIN ASSIGNMENTS
 **********************************************/

#define LEFT_HALL_EFFECT A3
#define RIGHT_HALL_EFFECT A4
#define LM35 A5
#define COMMON_INPUT_PIN 2
#define TRIG 3
#define ECHO 8
#define CLOCK_PIN 9
#define SERVO 10
#define SERVO_PEN 11
#define LATCH_PIN 12
#define DATA_PIN 13

/**********************************************
   variable to determine the functionality of the robot
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

Servo penservo;
/**********************************************
   CONSTANTS FOR THE MOTOR SHIELD
 **********************************************/
const float radius = 3.25 / 2; // assuming the radius is the same for two wheels
const int numOfMotors = 2;

// distance of an object in front of the robot that indicates
// when the robot should stop and slow down
const int TOO_CLOSE = 10;
const int SLOW_DOWN = 50 + 2 * M_PI * radius; // 50cm + circumference

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

  const int calInterval;
  const int timeout; //  in millis

  int prev_Reading;
  int rotationCounter;
  unsigned long prev_Millis;
  const unsigned long debounce_Millis;
  double distanceTravelled;
} MotorContrl; // define this struct as a new type called MotorControl

int E1Pin = 5;
int M1Pin = 4;
int E2Pin = 6;
int M2Pin = 7;

// initialize two motors
// E1 and M1 are right wheel, E2 and M2 are left wheel

MotorContrl Motors[] = {
  {E1Pin, M1Pin, RIGHT_HALL_EFFECT, 0, 0, 1, 5000, 1, 0, 0, 30, 0},
  {E2Pin, M2Pin, LEFT_HALL_EFFECT, 0, 0, 1, 5000, 1, 0, 0, 30, 0},
}; // create an array of MotorControl objects and initialize them 0

const int FORWARD = LOW;
const int BACKWARD = HIGH;
const int RIGHT_MOTOR = 0;
const int LEFT_MOTOR = 1;

// unsigned long bitPattern = 1<<17;
unsigned long bitPattern = 0x0c00; //pins 11&10 are the lever sensor and are interupt inabled
unsigned char numOfBits = 16;

//switch status
unsigned char sw1;
unsigned char sw2;
unsigned char sw3;
unsigned char sw4;
volatile unsigned char interrupt = 0;

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

  //attach servo to pin D9 and st its start postion to 90
  myservo.attach(SERVO);
  myservo.write(90);
  penservo.attach(SERVO_PEN);
  penservo.write(0);
  //penservo.write(47);
  pinMode(M1Pin, OUTPUT);
  pinMode(M2Pin, OUTPUT);
  setMotorDirection(RIGHT_MOTOR, FORWARD);
  //setMotorSpeed(RIGHT_MOTOR, 500);

  setMotorDirection(LEFT_MOTOR, FORWARD);
  //setMotorSpeed(LEFT_MOTOR, 500);


  //LCD & shift register STUFF********************************************

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

  // setup lcd
  setupLCD();
  blinkCurser(true);
  setCurserLocation(0, 0);
  writeLCDString("CPEN 291");
  setCurserLocation(1, 0);
  writeLCDString("Team G19");
  delay(2000);
  clearDisplay();

  //ensure the magnets are aligned at very beginning
  analogWrite(Motors[LEFT_MOTOR].enPin, 80);
  analogWrite(Motors[RIGHT_MOTOR].enPin, 80);
  updateRPMAndSpeed(LEFT_MOTOR);
  updateRPMAndSpeed(RIGHT_MOTOR);
  while (Motors[LEFT_MOTOR].distanceTravelled == 0 || Motors[RIGHT_MOTOR].distanceTravelled == 0) {
    updateRPMAndSpeed(LEFT_MOTOR);
    updateRPMAndSpeed(RIGHT_MOTOR);
    if (Motors[LEFT_MOTOR].distanceTravelled > Motors[RIGHT_MOTOR].distanceTravelled) {
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
      while (Motors[LEFT_MOTOR].distanceTravelled != Motors[RIGHT_MOTOR].distanceTravelled) {
        updateRPMAndSpeed(LEFT_MOTOR);
        updateRPMAndSpeed(RIGHT_MOTOR);
      }
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    }
    else if (Motors[LEFT_MOTOR].distanceTravelled < Motors[RIGHT_MOTOR].distanceTravelled) {
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      while (Motors[LEFT_MOTOR].distanceTravelled != Motors[RIGHT_MOTOR].distanceTravelled) {
        updateRPMAndSpeed(LEFT_MOTOR);
        updateRPMAndSpeed(RIGHT_MOTOR);
      }
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    }
  }

  clearDisplay();

  delay(5000);
  // Serial.println(bitPattern, BIN);
}

void loop()
{
  if (interrupt)
  
  {
    mask = 0;
    clearDisplay();
    setCurserLocation(0, 0);
    writeLCDString("Lever!");
    digitalShiftWrite(RED_LED, LOW);
    digitalShiftWrite(GREEN_LED, HIGH);
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    if (interrupt == 1) //front
    {
      setMotorDirection(LEFT_MOTOR, BACKWARD);
      setMotorDirection(RIGHT_MOTOR, BACKWARD);
      setCurserLocation(1, 0);
      writeLCDString("Move Back!");
      analogWrite(Motors[LEFT_MOTOR].enPin, 120);
      analogWrite(Motors[RIGHT_MOTOR].enPin, 120);
      for(int i=0; i<12; i++)
      { 
        buzz(BUZZER, 120);
        delay(120);
      }
      setMotorDirection(LEFT_MOTOR, FORWARD);
      setMotorDirection(RIGHT_MOTOR, FORWARD);
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      delay(1800);
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    }
    else //back
    {
      setMotorDirection(LEFT_MOTOR, FORWARD);
      setMotorDirection(RIGHT_MOTOR, FORWARD);
      setCurserLocation(1, 0);
      writeLCDString("Move Forward!");
      analogWrite(Motors[LEFT_MOTOR].enPin, 120);
      analogWrite(Motors[RIGHT_MOTOR].enPin, 120);
      for(int i=0; i<12; i++)
      { 
        buzz(BUZZER, 120);
        delay(120);
      }
      setMotorDirection(LEFT_MOTOR, BACKWARD);
      setMotorDirection(RIGHT_MOTOR, BACKWARD);
      analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
      delay(1800);
      analogWrite(Motors[LEFT_MOTOR].enPin, 0);
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
      writeLCDString("Principle");
      setCurserLocation(1, 0);
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
      writeLCDString("Principle");
      setCurserLocation(1, 0);
      writeLCDString("Function 2");
      mask = 1;
    }
    principal_function2();
  }
  else if (mode == AF && !interrupt)
  {
    if (!mask)
    {
      clearDisplay();
      setCurserLocation(0, 0);
      writeLCDString("Additional");
      setCurserLocation(1, 0);
      writeLCDString("Function");
      mask = 1;
    }
    additional_function();
  }
  else if (!mask && !interrupt)
  {
    clearDisplay();
    setCurserLocation(0, 0);
    writeLCDString("ERROR: Set Mode");
    setCurserLocation(1, 0);
    writeLCDString("switch & restart");
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

//interuppt service rutine
void doISR()
{
  int lever1 = digitalShiftRead(REG_INPUT_LEVER1);
  int lever2 = digitalShiftRead(REG_INPUT_LEVER2);
  if (lever1 || lever2)
  {
    digitalShiftWrite(RED_LED, HIGH);
    digitalShiftWrite(GREEN_LED, LOW);
  }
  if (lever1)
    interrupt = 1;
  if (lever2)
    interrupt = 2;
}
/**************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE MOTOR
 **************************************************************************************************************************************************************************************/
/**
   Use hall effect to update the RPM and Speed of the wheel
*/
void updateRPMAndSpeed(int motorNum)
{
  int curr_Reading = digitalRead(Motors[motorNum].sensorPin);

  if (curr_Reading == 1 && Motors[motorNum].prev_Reading == 0)
  {
    // rising edge
    unsigned long curr_Millis = millis();
    unsigned long delta_Millis = curr_Millis - Motors[motorNum].prev_Millis;
    Motors[motorNum].distanceTravelled += radius * 3.14;
    Serial.println("This is " + (String)motorNum + "     " + (String)Motors[motorNum].distanceTravelled);
    if (delta_Millis > Motors[motorNum].debounce_Millis)
    {
      Motors[motorNum].rotationCounter++;

      if (Motors[motorNum].rotationCounter == 1)
        Motors[motorNum].prev_Millis = curr_Millis;
    }

    if (Motors[motorNum].rotationCounter - 1 == Motors[motorNum].calInterval)
    {
      Motors[motorNum].RPM = 60 * 1000.0 / (delta_Millis / (Motors[motorNum].rotationCounter - 1));
      Motors[motorNum].Speed = rpmToSpeed(Motors[motorNum].RPM, radius); // again, assuming the radius is the same for two wheels
      // update speed as well
      Motors[motorNum].prev_Millis = curr_Millis;
      Motors[motorNum].rotationCounter = 1;
      // for debugging purposes
      /*
        Serial.println(String(motorNum == 1 ? "left " : "right ") + "wheel: ");
        Serial.println("RPM: " + String(Motors[motorNum].RPM));
        Serial.println("Speed: " + String(Motors[motorNum].Speed));
        Serial.println("counter: " + String(Motors[motorNum].rotationCounter));
        Serial.println();
      */
      flag = !flag;
    }
  }
  if (millis() -  Motors[motorNum].prev_Millis > Motors[motorNum].timeout)
  {
    resetRPMAndSpeed(motorNum);
    /*
      Serial.println(String(motorNum == 1 ? "left " : "right ") + "wheel: ");
      Serial.println("RPM: " + String(Motors[motorNum].RPM));
      Serial.println("Speed: " + String(Motors[motorNum].Speed));
      Serial.println("counter: " + String(Motors[motorNum].rotationCounter));
      Serial.println();
    */
    flag = !flag;
  }
  Motors[motorNum].prev_Reading = curr_Reading;
}
void updateLoop(int motorNum) {
  while (flag) {
    updateRPMAndSpeed(motorNum);
  }
  flag = !flag; //flag is now true
}
/**
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

/**
   PostCond. UPDATES MOTOR SPEED
   @params:
          motorNum, a number represents the motor whose speed is going to be set
          0 is the right motor, and 1 is the left one.

          speed in cm/sec
*/
void setMotorSpeed(int motorNum, int speedM)
{
  if (speedM <= 0) {
    analogWrite(Motors[motorNum].enPin, 0);
    return;
  }
  else {
    if (motorNum == RIGHT_MOTOR) {
      if (speedM >= 50.5) //if speed is greater than max, set max
        analogWrite(Motors[motorNum].enPin, 255);
      else
        analogWrite(Motors[motorNum].enPin, (int) exp((speedM + 87.81) / 25.407));
    }
    else {
      if (speedM >= 55.1)
        analogWrite(Motors[motorNum].enPin, 255);
      else
        analogWrite(Motors[motorNum].enPin, (int) exp((speedM + 95.108) / 27.237));
    }
  }
  //updateLoop(motorNum); CHANGED, MUST UPDATE MANUALLY*********************************************************
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

//accelerometer
//02/21/18
/**
   Tells robot to go straight. Do this be compensating left and right
*/
void keepStraight(int PWM) {
  //LEFT MOTOR IS MASTER, RIGHT MOTOR IS SLAVE (Note: right motor rotates faster than left)
  if (PWM <= 0) {
    analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    return;
  }
  if (readDistance() <= TOO_CLOSE)
    return;

  updateRPMAndSpeed(LEFT_MOTOR);
  updateRPMAndSpeed(RIGHT_MOTOR);
  analogWrite(Motors[RIGHT_MOTOR].enPin, PWM);
  analogWrite(Motors[LEFT_MOTOR].enPin, PWM);
  updateRPMAndSpeed(LEFT_MOTOR);
  updateRPMAndSpeed(RIGHT_MOTOR);


  if (Motors[LEFT_MOTOR].distanceTravelled < Motors[RIGHT_MOTOR].distanceTravelled) {
    analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    while (Motors[LEFT_MOTOR].distanceTravelled < Motors[RIGHT_MOTOR].distanceTravelled) {
      updateRPMAndSpeed(LEFT_MOTOR);
      updateRPMAndSpeed(RIGHT_MOTOR);
    }
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    delay(100);
    analogWrite(Motors[RIGHT_MOTOR].enPin, PWM);
    analogWrite(Motors[LEFT_MOTOR].enPin, PWM);
  }

  else if (Motors[LEFT_MOTOR].distanceTravelled > Motors[RIGHT_MOTOR].distanceTravelled) {
    analogWrite(Motors[LEFT_MOTOR].enPin, 0);
    while (Motors[LEFT_MOTOR].distanceTravelled > Motors[RIGHT_MOTOR].distanceTravelled) {
      updateRPMAndSpeed(LEFT_MOTOR);
      updateRPMAndSpeed(RIGHT_MOTOR);
    }
    analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
    delay(100);
    analogWrite(Motors[LEFT_MOTOR].enPin, PWM);
    analogWrite(Motors[RIGHT_MOTOR].enPin, PWM);
  }

  else {
    analogWrite(Motors[RIGHT_MOTOR].enPin, PWM);
    analogWrite(Motors[LEFT_MOTOR].enPin, PWM);
  }
  updateRPMAndSpeed(LEFT_MOTOR);
  updateRPMAndSpeed(RIGHT_MOTOR);
}
/**
   Slows down robot to a stop. Not it will loop until it is too close.
   PreCond.
    Robot is moving forward
   PostCond.
    Robot will stop at a distance TOO_CLOSE to closest object infront of it
*/
void slowDown() {
  currentDistance = readDistance();
  int slowSpeed = 160;
  while (currentDistance >= TOO_CLOSE && slowSpeed >= 0) {
    /*
      setMotorSpeed(LEFT_MOTOR,Motors[LEFT_MOTOR].Speed/exp(fabs(SLOW_DOWN-currentDistance))); //if slowing too fast use a set speed
      setMotorSpeed(RIGHT_MOTOR,Motors[LEFT_MOTOR].Speed/exp(fabs(SLOW_DOWN-currentDistance)));
      delay(50);
      //keepStraight();
      delay(50);
    */
    keepStraight(slowSpeed);
    slowSpeed -= 20;
    currentDistance = readDistance();
    //if(Motors[RIGHT_MOTOR].Speed == 0 || Motors[LEFT_MOTOR].Speed == 0)
    //break;
  }
  //stop robot
  //setMotorSpeed(LEFT_MOTOR,0);
  //setMotorSpeed(RIGHT_MOTOR,0);
  analogWrite(Motors[RIGHT_MOTOR].enPin, 0);
  analogWrite(Motors[LEFT_MOTOR].enPin, 0);
}
/**
   Turns left or right
   PreCond.
    Robot must be not moving
    PostCond.
     Robot is still not moving but now turned in selected direction

     @param
        l_or_r: determine whether to turn left or right. 0 for right, 1 for left.
*/
void turn(int l_or_r) {
  //resetRPMAndSpeed(l_or_r); //reset to get clean rotation counter
  setMotorDirection(l_or_r, BACKWARD); //tell the selected motor to go backwards.

  analogWrite(Motors[l_or_r].enPin, 120);
  //updateLoop(Motors[l_or_r]);
  delay(1800);
  analogWrite(Motors[l_or_r].enPin, 0);
  //if(l_or_r == 0){ //right
  /*
    analogWrite(Motors[RIGHT_MOTOR].enPin,120);
    analogWrite(Motors[LEFT_MOTOR].enPin,120);
    delay(800);
    analogWrite(Motors[RIGHT_MOTOR].enPin,0);
    analogWrite(Motors[LEFT_MOTOR].enPin,0);
    /*

    while(Motors[l_or_r].rotationCounter < 3){ //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! measure this shit to determine how much u got to turn 90 degrees
    updateLoop(l_or_r);
    }
  */
  //setMotorSpeed(l_or_r,0); //stop robot
  setMotorDirection(l_or_r, FORWARD);
}
/***
   Checks whether to go left or right; one with most distance. Goes right if equal distance. Returns 0 for right, 1 for left
*/
int check_l_or_r() {
  readBothSides();
  return rightDistance >= leftDistance ? 0 : 1;
}
