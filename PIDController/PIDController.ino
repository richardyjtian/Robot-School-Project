#include <PID_v1.h>
#include <Servo.h>
#include <math.h>



/*******************************************************************
   PIN ASSIGNMENTS FOR DIP SWITCH
 ******************************************************************/
const int pf1 = 1, pf2 = 2, af = 3;
int pf1v = LOW, pf2v = LOW, afv = LOW;

/*******************************************************************
   PIN ASSIGNMENTS FOR LCD
 ******************************************************************/
// symbolic constants for the pins that provide input to the LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;

/**********************************************
   PIN ASSIGNMENTS FOR THE ULTRASONIC SENSOR, LM35, AND THE SERVO MOTOR
 **********************************************/
#define LM35 A0  //Pin A0
#define ECHO 13  // Pin D13
#define TRIG 12  // Pin D12
#define SERVO 10 //Pin D9
#define VCC 11   //vcc pin for URF
#define RIGHT_HALL_EFFECT 2
#define LEFT_HALL_EFFECT 3




/**********************************************
   GLOBAL VARIABLES FOR THE ULTRASONIC SENSOR AND SERVO
 **********************************************/
Servo myservo;
float rightDistance;
float leftDistance;
float currentDistance;

/**********************************************
   CONSTANTS FOR THE MOTOR SHIELD
 **********************************************/
const float radius = 3.25/2; // assuming the radius is the same for two wheels
const int numOfMotors = 2;

// distance of an object in front of the robot that indicates
// when the robot should stop and slow down
const int TOO_CLOSE = 5; // 5cm
const int SLOW_DOWN = 55; // 55 cm

//individual motors of the motor shield
// using struct to store infor about two motors and control them
// get help from: https://www.dfrobot.com/wiki/index.php/2A_Motor_Shield_For_Arduino_Twin)_(SKU:DRI0017)





typedef struct
{
  const int enPin;
  const int directionPin;
  const int sensorPin;

  double RPM;
  double Speed;

  const int calInterval;
  const int timeout; //  in millis

  int prev_Reading;
  int rotationCounter;
  unsigned long prev_Millis;
  const unsigned long debounce_Millis;

  double setSpeed;
  double PWM;

} MotorContrl; // define this struct as a new type called MotorControl

int E1Pin = 5;
int M1Pin = 4;
int E2Pin = 6;
int M2Pin = 7;

// initialize two motors
// E1 and M1 are right wheel, E2 and M2 are left wheel

MotorContrl Motors[] = {
  {E1Pin, M1Pin, RIGHT_HALL_EFFECT, 0, 0, 1, 3000, 1, 0, 0, 30, 0, 0},
  {E2Pin, M2Pin, LEFT_HALL_EFFECT, 0, 0, 1, 3000, 1, 0, 0, 30, 0, 0},
}; // create an array of MotorControl objects and initialize them 0

const int FORWARD = LOW;
const int BACKWARD = HIGH;
const int RIGHT_MOTOR = 0;
const int LEFT_MOTOR = 1;

const int TURN_LEFT = 0;
const int TURN_RIGHT = 1;


const float turningRadius = 8.5 - 2; // uneducated guess









/**************************************************************************************************************************************************************************************
   LIBRARY FUNCTIONS FOR THE MOTOR
 **************************************************************************************************************************************************************************************/

/**
   @params:
          radius of the wheel in cm
   @return:
          speed in cm/sec
*/
double rpmToSpeed(double rpm, float r)
{
  return rpm * 2 * M_PI * r / 60;
}



double speedToRpm(double cmPerSec, float r)
{
  return cmPerSec * 60 / ( 2 * M_PI * r );
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
void setMotorSpeedByPWM(int motorNum, int pwm)
{
  if (pwm > 255)
    pwm = 255;
  else if (pwm <= 0)
    pwm = 0;

  analogWrite(Motors[motorNum].enPin, pwm);
}

/**void
   @params:
          motorNum, a number represents the motor whose speed is going to be set
          0 is the right motor, and 1 is the left one.

          direction, either FORWARD or BACKWARD
*/
void setMotorDirection(int motorNum, int direction)
{
  digitalWrite(Motors[motorNum].directionPin, direction);
}


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
      // Serial.println(String(motorNum == 1 ? "left " : "right ") + "wheel: ");
      // Serial.println("RPM: " + String(Motors[motorNum].RPM));
      // Serial.println("Speed: " + String(Motors[motorNum].Speed));
      // Serial.println("counter: " + String(Motors[motorNum].rotationCounter));

      //  Serial.println(String(Motors[motorNum].Speed));
      //  Serial.print(" ");
    }
  }

  if (millis() -  Motors[motorNum].prev_Millis > Motors[motorNum].timeout)
  {
    resetRPMAndSpeed(motorNum);
    Motors[motorNum].prev_Millis = millis();
    // Serial.println(String(motorNum == 1 ? "left " : "right ") + "wheel: ");
    // Serial.println("RPM: " + String(Motors[motorNum].RPM));
    // Serial.println("Speed: " + String(Motors[motorNum].Speed));
    // Serial.println("counter: " + String(Motors[motorNum].rotationCounter));
    // Serial.println();

    // Serial.println(String(Motors[motorNum].Speed));
    // Serial.print(" ");

  }
  Motors[motorNum].prev_Reading = curr_Reading;
}




void setMotorSpeed(int motorNum, double cmPerSec) {
  Motors[motorNum].setSpeed = cmPerSec;
}









// Kp, Ki, Kd are tuning params


 //PID RightPID(&Motors[RIGHT_MOTOR].Speed, &Motors[RIGHT_MOTOR].PWM, &Motors[RIGHT_MOTOR].setSpeed, 4, 2, 0.3, DIRECT);
 //PID LeftPID(&Motors[LEFT_MOTOR].Speed, &Motors[LEFT_MOTOR].PWM, &Motors[LEFT_MOTOR].setSpeed, 5, 2.5, 0.5, DIRECT);


PID RightPID(&Motors[RIGHT_MOTOR].Speed, &Motors[RIGHT_MOTOR].PWM, &Motors[RIGHT_MOTOR].setSpeed, 4, 0, 0, DIRECT);
PID LeftPID(&Motors[LEFT_MOTOR].Speed, &Motors[LEFT_MOTOR].PWM, &Motors[LEFT_MOTOR].setSpeed, 5, 0, 0, DIRECT);


void setup()
{



  /******************************
     Setup for LCD
   ******************************/

  Serial.begin(9600);
  delay(1000);

  //setup the servo motor, lm35, and the ultrasonic sensor
  //use pin 11 as a powersource
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  pinMode(LM35, INPUT);   // Set LM35 to input
  pinMode(ECHO, INPUT);   // Set ECHO to input
  pinMode(TRIG, OUTPUT);  // Set TRIG to output
  pinMode(SERVO, OUTPUT); // Set SERVO to output

  pinMode(RIGHT_HALL_EFFECT, INPUT);
  pinMode(LEFT_HALL_EFFECT, INPUT);

  //attach servo to pin D9 and st its start postion to 90
  myservo.attach(SERVO);
  myservo.write(90);




  /******************************
     Setup for PID Controller
   ******************************/


  double setSpeed = 10;
  double rightMinPWM = 0;
  double rightMaxPWM = 255;

//  RIGHT MOTOR
  setMotorDirection(RIGHT_MOTOR, FORWARD);
  setMotorSpeed(RIGHT_MOTOR, setSpeed);
  RightPID.SetOutputLimits(rightMinPWM, rightMaxPWM);
  RightPID.SetMode(AUTOMATIC);





  double leftMinPWM = 0;
  double leftMaxPWM = 255;

  // LEFT MOTOR
  setMotorDirection(LEFT_MOTOR, FORWARD);
  setMotorSpeed(LEFT_MOTOR, setSpeed);
  LeftPID.SetOutputLimits(leftMinPWM, leftMaxPWM);
  LeftPID.SetMode(AUTOMATIC);



}

void loop()
{


  updateRPMAndSpeed(RIGHT_MOTOR);
  updateRPMAndSpeed(LEFT_MOTOR);

  RightPID.Compute();
  LeftPID.Compute();
  setMotorSpeedByPWM(RIGHT_MOTOR, (int) Motors[RIGHT_MOTOR].PWM);
  setMotorSpeedByPWM(LEFT_MOTOR, (int) Motors[LEFT_MOTOR].PWM);


  Serial.print(String(Motors[RIGHT_MOTOR].Speed));
  Serial.print(",");
  Serial.print(String(Motors[LEFT_MOTOR].Speed));
  Serial.print(",");
//    Serial.print(String(Motors[LEFT_MOTOR].PWM));
//  Serial.print(",");
  Serial.print(String(Motors[LEFT_MOTOR].setSpeed));

  Serial.println("");




  //  if (Motors[RIGHT_MOTOR].Speed == 0) {
  //    Motors[RIGHT_MOTOR].Speed = setSpeed / 3;
  //  }

  //    Serial.println(String(Motors[RIGHT_MOTOR].PWM));
  //    Serial.print(" ");
  // setMotorSpeed(LEFT_MOTOR, 40);
  // updateRPMAndSpeed(LEFT_MOTOR);
  // LeftPID.Compute();
  // correctPWM();
}



void turn(int direction, int degree) {

  int pivotMotorNum = direction == TURN_LEFT ? LEFT_MOTOR : RIGHT_MOTOR;
  int turningSpeed = pivotMotorNum == LEFT_MOTOR ? (Motors[RIGHT_MOTOR].Speed) : (Motors[LEFT_MOTOR].Speed);
  int curveLength = turningRadius * 2 * M_PI * degree / 360;

}


// archive

// void syncSpeed(float RPMtolerance, int referenceMotor, int anotherMotor, float changeCoeff)
// {

//     while (true)
//     {
//         float referenceRPM = Motors[referenceMotor].RPM;
//         float anotherRPM = Motors[anotherMotor].RPM;

//         float diff = anotherRPM - referenceRPM;

//         if( abs(diff) <= RPMtolerance )
//             break;

//         if(diff > 0){
//             // another motor needs to slow down
//             int newSpeedPercentange = -diff * changeCoeff + Motors[anotherMotor].speedPercentage;
//             setMotorSpeed(anotherMotor, newSpeedPercentange);

//         }else if( diff < 0 && (Motors[anotherMotor].speedPercentage == Motors[referenceMotor].speedPercentage) ){
//             // reference motor needs to slow down the maximum speed of the other motor is still lower than that of reference motor
//             int newSpeedPercentange = -diff * changeCoeff + Motors[referenceMotor].speedPercentage;
//             setMotorSpeed(referenceMotor, newSpeedPercentange);

//         }else{
//             // another motor needs to speed up
//             int newSpeedPercentange = diff * changeCoeff + Motors[anotherMotor].speedPercentage;
//             setMotorSpeed(anotherMotor, newSpeedPercentange);
//         }

//         updateRPMAndSpeed(LEFT_MOTOR);
//         updateRPMAndSpeed(RIGHT_MOTOR);

//     }
// }




// void correctPWM(){
//     for(int i = 0; i < numOfMotors; i++){
//         setMotorSpeedByPWM(i, (int) Motors[i].PWM);
//     }
// }


