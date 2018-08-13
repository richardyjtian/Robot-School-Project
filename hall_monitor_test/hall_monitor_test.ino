
#define HALLEFFECT 2
void setup()
{
  Serial.begin(9600);
}


void loop()
{

}

int prev_read = 0;
int time1 = 0;
int time2 = 0;
int timePerRotation = 0;
int reading;
int i = 0;

int getRmp(int hallpin, int numReadings)
{
  for(i=0;i<numReadings;i++)
  {
    reading = digitalRead(hallPin);
    if (reading == 1 && prev_read == 0) {
      time1 = time2;
      time2 = millis();
      if (fabs(timePerRotation - (time2 - time1)) <= 5)
        return 6000 / (time2 - time1);
      timePerRotation = time2 - time1;
    }
    prev_read = reading;
  }
  return 6000/timePerRotation;
}

// 
int getRpm(int hallPin)
{
  timeReading = readTimePerRotation(hallPin, 5);
  

  
}

