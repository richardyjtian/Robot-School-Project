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
