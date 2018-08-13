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
#define HALL_EFFECT_1 9
#define HALL_EFFECT_2 10

#define LM35 5 // A5

#define COMMON_INPUT_PIN 2
#define LATCH_PIN 12
// #define LATCH_PIN 4
#define CLOCK_PIN 11
// #define CLOCK_PIN 3
#define DATA_PIN 13
// #define DATA_PIN 5
#define ECHO 8

// unsigned long bitPattern = 1<<17;
unsigned long bitPattern = 0xfc00;
int numOfBits = 16;

void clearBits()
{
  for (int i = 0; i < numOfBits; i++)
  {
    digitalShiftWrite(i, LOW);
  }
}

// display a number on the digital segment display
void digitalShiftWrite(int pin, int value)
{

  // it would be something like 00010000 or 11101111
  unsigned long bits = (value == HIGH) ? (1 << pin) : ~(1 << pin) | 0xfc00;
  // bits += (1<<17);
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

// display a number on the digital segment display
int digitalShiftRead(int pin)
{
  int bits = pow(2.0, pin);
  // set the LATCH_PIN to low potential, before sending data
  digitalWrite(LATCH_PIN, LOW);

  // the original data (bit pattern)
  // assuming that this bit is the only bit set to 1 among all bits used as inputs
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bits | bitPattern);

  // set the LATCH_PIN to high potential, after sending data
  digitalWrite(LATCH_PIN, HIGH);

  delay(2); // delay for debouncing
  int result = digitalRead(COMMON_INPUT_PIN);

  // restore the old bit pattern (dont really need to do this, cuz next time we other bits used as inputs will be set to zero)
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitPattern);
  digitalWrite(LATCH_PIN, HIGH);
  delay(2);

  return result;
}

// LCD library

const int LCDPins[] = {D4, D5, D6, D7};

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

void setup()
{

  Serial.begin(9600);
  delay(100);

  Serial.println(bitPattern, BIN);

  // put your setup code here, to run once:
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(COMMON_INPUT_PIN, INPUT);

  clearBits();

  // setup lcd
  // put your setup code here, to run once:
  setupLCD();
  blinkCurser(true);
  setCurserLocation(0, 0);
  writeLCDString("CPEN 291");
  setCurserLocation(1, 0);
  writeLCDString("Team L2B-5C");
  delay(2200);
  clearDisplay();
  displayNumber(6666);

  Serial.println(bitPattern, BIN);
}

void loop()
{
  //    delay(1000);
  

  //     Serial.println(String(digitalRead(HALL_EFFECT_2)));
  //Serial.println(String(analogRead(LM35) * 500.0 / 1024));
  //Serial.println(String(digitalRead(COMMON_INPUT_PIN)));

  digitalShiftWrite(BUZZER, HIGH);

  delay(5);
  digitalShiftWrite(BUZZER, LOW);


//  digitalShiftWrite(8, HIGH);
//  digitalShiftWrite(9, HIGH);
//  delay(2000);
//
//  Serial.println(bitPattern, BIN);
//
//
//  digitalShiftWrite(9, LOW);
//  digitalShiftWrite(8, LOW);
//  delay(2000);


//  Serial.println(digitalRead(COMMON_INPUT_PIN));


}
