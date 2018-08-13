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
