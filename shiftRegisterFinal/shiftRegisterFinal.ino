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


// arduino pins
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

unsigned char mode; // 1 for pf1 - 2 for pf2 - 3 for af

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(COMMON_INPUT_PIN, INPUT);
  clearBits();
  if(digitalShiftRead(SW4)) // only attach interrupt if sw4 is on
    attachInterrupt(digitalPinToInterrupt(COMMON_INPUT_PIN), check, RISING); 
  //mode = setMode();

  // setup lcd
  setupLCD();
  blinkCurser(true);
  setCurserLocation(0, 0);
  writeLCDString("CPEN 291");
  setCurserLocation(1, 0);
  writeLCDString("Team L2B-5C");
  
  Serial.println(digitalShiftRead(SW1));
  Serial.println(digitalShiftRead(SW2));
  Serial.println(digitalShiftRead(SW3));
  Serial.println(digitalShiftRead(SW4));
  Serial.println("-----------------");
  digitalShiftWrite(GREEN_LED, HIGH);
  delay(2000);
  buzz(BUZZER, 5000);
  digitalShiftWrite(GREEN_LED, LOW);
  digitalShiftWrite(RED_LED, HIGH);
  delay(5000);
  digitalShiftWrite(RED_LED, LOW);

  Serial.println(digitalShiftRead(SW1));
  Serial.println(digitalShiftRead(SW2));
  Serial.println(digitalShiftRead(SW3));
  Serial.println(digitalShiftRead(SW4));
}

//interuppt service rutine
void check()
{
  int lever1 = digitalShiftRead(REG_INPUT_LEVER1); 
  int lever2 = digitalShiftRead(REG_INPUT_LEVER2);
    
  if(lever1)
  {
    //turn left or right 
    Serial.println("lever1");
  }
  if(lever2)
  {
    //turn left or right 
    Serial.println("lever2");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}

// Shift Register pin read/write liberary------------------------------------------------------------------------------------------------------------------
// unsigned long bitPattern = 1<<17;
unsigned long bitPattern = 0x0c00; //pins 11&10 are the lever sensor and are interupt inabled  
unsigned char numOfBits = 16;

//set all pins on shift register to 0
void clearBits()
{
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
  digitalWrite(LATCH_PIN, HIGH);
}

// read pin from pin needs to be from 9-15 other wise code will be comprimised 
int digitalShiftRead(int pin) 
{
  unsigned long temp = bitPattern;
  temp = temp&0x00ff;
  temp = temp | (1<<pin);
  
  // set the LATCH_PIN to low potential, before sending data
  digitalWrite(LATCH_PIN, LOW);

  // the original data (temp part of bitpattern)
  // assuming that this bit is the only bit set to 1 among all bits used as inputs
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, ((temp & 0xff00) >> 8)); //send upper 8 bits
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, (temp & 0x00ff)); //send lower 8 bits

  // set the LATCH_PIN to high potential, after sending data
  digitalWrite(LATCH_PIN, HIGH);

  delay(10); // delay for debouncing and for the shift register to update
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

// LCD library ----------------------------------------------------------------------------------------------------------------------------------------------
/* Functions include:
 * void displayNumber(int num)
 * void shiftCurser(bool right)
 * void shiftDisplay(bool right)
 * void curserHome() //Returns the cursor to the home position (Address 0). Returns display to its original state if it was shifted.
 * void setCurserLocation(int row, int col)
 * void blinkCurser(bool doesBlink)
 * void writeLCDString(String s)
 * void writeLCD(char character)
 * void setupLCD()
 * void clearDisplay()
 */

const unsigned char LCDPins[] = {D4, D5, D6, D7};

//got help from https://stackoverflow.com/questions/36274902
void displayNumber(int num)
{
  char number[sizeof(int) * 4 + 1]; // plus 1 for \0
  sprintf(number, "%d", num);
  writeLCDString(number);
}

void shiftCurser(bool right)
{ //true to right shift false for left shift
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
{ //true to right shift false for left shift
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
{ //Returns the cursor to the home position (Address 0). Returns display to its original state if it was shifted.
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

// other functions --------------------------------------------------------

unsigned char setMode()
{
  if(digitalShiftRead(SW1))
    return 1;
  if(digitalShiftRead(SW2))
    return 2;
  if(digitalShiftRead(SW3))
    return 3;
  return 0;
}

// buzzes the said pin for the given period in miliseconds
void buzz(int pin, int miliseconds)
{
  Serial.println("buzzing");
  for(int i=0; i < miliseconds; i+=2)
  {
    digitalShiftWrite(pin, LOW);
    delay(1);
    digitalShiftWrite(pin, HIGH);
    delay(1);
  }
  digitalShiftWrite(pin, LOW);
}

