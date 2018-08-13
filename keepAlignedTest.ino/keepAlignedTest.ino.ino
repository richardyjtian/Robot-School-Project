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

#define LEFT_HALL_EFFECT 17 //a3
#define RIGHT_HALL_EFFECT 18 //a4
#define LM35 A5
#define COMMON_INPUT_PIN 2
//#define TRIG 3
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
const int TOO_CLOSE = 20;
const int SLOW_DOWN = 75 + 2 * M_PI * radius; // 50cm + circumference

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

const int FORWARD = LOW;
const int BACKWARD = HIGH;
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
  myservo.write(80); //needs to be called twice
  delay(330);
  myservo.write(90);
  delay(330);
  penservo.attach(SERVO_PEN);
  penservo.write(0);
  delay(330);
  pinMode(M1Pin, OUTPUT);
  pinMode(M2Pin, OUTPUT);

  //LCD & shift register STUFF********************************************

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(COMMON_INPUT_PIN, INPUT);
  delay(1000);

  align();
  delay(3000);


}

void loop()
{
  keepAligned(240, FORWARD, FORWARD);
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


/**************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE MOTOR
 **************************************************************************************************************************************************************************************/
//ALLIGNS THE TWO WHEELS SO THAT THE HALL EFFEC SENSORS ARE IN THE SAME POSITION S
void align()
{

  int lastRh = digitalRead(RIGHT_HALL_EFFECT);
  int lastLh = digitalRead(LEFT_HALL_EFFECT);
  int rh;
  int lh;
  setMotorDirection(RIGHT_MOTOR, FORWARD);
  setMotorDirection(LEFT_MOTOR, FORWARD);
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

