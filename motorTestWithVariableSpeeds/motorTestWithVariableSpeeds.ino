//Arduino PWM Speed Control：
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;
int i;

void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
}

void loop()
{

  if(Serial.available() >0)
    writeMotor();
}

void writeMotor()
{
  // read the incoming byte:
  i = Serial.parseInt();

  // say what you got:
  Serial.print("PWM: ");
  Serial.println(i, DEC);
  analogWrite(E2, i);   //PWM Speed Control
  analogWrite(E1, i);   //PWM Speed Control
}

