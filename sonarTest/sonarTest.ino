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

#define RS 1
#define E 2
#define D4 3
#define D5 4
#define D6 5
#define D7 6

/**********************************************
   MASTER PIN ASSIGNMENTS
 **********************************************/

#define LEFT_HALL_EFFECT 17  //a3
#define RIGHT_HALL_EFFECT 18 //a4
#define LM35 A5
#define COMMON_INPUT_PIN 2

#define TRIG 3
#define ECHO 8

#define CLOCK_PIN 9
#define SERVO 9
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
unsigned int interval = 0;


/**********************************************
   CONSTANTS FOR THE MOTOR SHIELD
 **********************************************/
const float radius = 3.25 / 2; // assuming the radius is the same for two wheels
const int numOfMotors = 2;

// distance of an object in front of the robot that indicates
// when the robot should stop and slow down
const int TOO_CLOSE = 20;
const int SLOW_DOWN = 50; // 50cm + circumference

//individual motors of the motor shield
// using struct to store infor about two motors and control them
// get help from: https://www.dfrobot.com/wiki/index.php/2A_Motor_Shield_For_Arduino_Twin)_(SKU:DRI0017)

// boolean flag = true;

float readDistance()
{
  // datasheet: https://www.robotshop.com/media/files/pdf2/hc-sr04-ultrasonic-range-finder-datasheet.pdf
  float speedOfSound;
  int numOfMeasurements = 20;
  float usPerMeter;
  float temp;
  float distance;

  float sum = 0;
  for (int i = 0; i < numOfMeasurements; i++)
  {
    //calculate temp from LM35 reading
    // temp = analogRead(LM35) * 500.0 / 1024; //read LM35 and use constant from https://create.arduino.cc/projecthub/TheGadgetBoy/making-lcd-thermometer-with-arduino-and-lm35-36-c058f0
    //  float temp = 23;
    // temp = 30; //read LM35 and use constant from https://create.arduino.cc/projecthub/TheGadgetBoy/making-lcd-thermometer-with-arduino-and-lm35-36-c058f0

    //determine constants for calculating distance
    speedOfSound = 343.0;
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
    else if (distance < 2)
      distance = 2;

    sum += distance;

    delay(60); // cycle period recommended by the datasheet
  }

  distance = sum / numOfMeasurements;
  if (distance > 400)
    distance = 400.0;
  else if (distance < 2)
    distance = 2.0;
    
  return distance;
}


void readBothSides()
{
  myservo.write(90);
  rotate90Deg(0);
  delay(500);
  leftDistance = readDistance();
  delay(500);
  rotate90Deg(1);
  delay(700);
  rightDistance = readDistance();
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




void setup()
{
  Serial.begin(9600);

  //setup the servo motor, lm35, and the ultrasonic sensor

  // pinMode(LM35, INPUT);       // Set LM35 to input
  pinMode(ECHO, INPUT);       // Set ECHO to input
  pinMode(TRIG, OUTPUT);      // Set TRIG to output
  pinMode(SERVO, OUTPUT);     // Set SERVO to output
  // pinMode(SERVO_PEN, OUTPUT); // Set SERVO_PEN to output
  //  pinMode(13, OUTPUT);
  //  delay(500);
  //  digitalWrite(13, HIGH); // VCC pin for servo

  myservo.attach(SERVO);
  delay(1000);
  myservo.write(90);
  delay(1000);
}

void loop()
{
  //  readBothSides();
  //  Serial.println("LEFT: "+ String(leftDistance));
  //  Serial.println("RIGHT: " + String(rightDistance));
  //  Serial.println("");
  //
  //  if (leftDistance < rightDistance)
  //    // keepAligned(160, BACKWARD, FORWARD);
  //  Serial.println("Turn Right");
  //
  //  else
  //    // keepAligned(160, FORWARD, BACKWARD);
  //  Serial.println("Turn Left");
  //
  //
  //  delay(2000);


  Serial.println(String(readDistance()));
  delay(500);
}
