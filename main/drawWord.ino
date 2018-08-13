//delay needed for 360
const float spin_delay = 3800;
//integer > 10
const float font_size = 2;

//integer > 0
int space = 7;

bool pen_up = true;

float prev_length;

void drawWord(char *word){
  if(strlen(word) > 1){
    for(int i = 0; i < strlen(word); i++){
      drawWord(word[i]);
    }  
    return;
  }
  float a_hyp = font_size/cos((float)15*(2*M_PI/360)); //a hypotenuse
  Serial.println(cos((float)15*(2*M_PI/360)));
  switch(word[0]){
    case 'a':
    case 'A':
      rotate(345);
      lowerPen();
      drawLine(a_hyp, true, 1/2);
      rotate(105);
      lowerPen();
      drawLine(a_hyp*cos((float)75*(2*M_PI/360)), false, 0.0);
      rotate(105);
      lowerPen();
      drawLine(a_hyp/2, true, 1.0);
      lowerPen();
      drawLine(a_hyp/2, false, 0.0);
      rotate(75);
      drawLine(space, false, 0.0);
      rotate(90);
      clearDisplay();
      writeLCDString("DONE");
    return;
    
    case 'b':
    case 'B'://///////////////////////////////////////////////////////
    return;
    
    case 'c':
    case 'C'://///////////////////////////
    return;
    
    case 'd':
    case 'D':////////////////////////////
    return;
    
    case 'e':
    case 'E':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/3, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      liftPen();
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'f':
    case 'F':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/3, true, 1.0);
      rotate(90);
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(space + font_size/2, false, 0.0);
      rotate(90);
    return;
    
    case 'g':
    case 'G':////////////////////////////////////////////////////////
    return;
    
    case 'h':
    case 'H':
      lowerPen();
      drawLine(font_size, true, 1/2);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;

    case '1':
    case 'i':
    case 'I':
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'j':
    case 'J':////////////////////////////
    return;
    
    case 'k':
    case 'K':
      lowerPen();
      drawLine(font_size, true, 1/2);
      rotate(335);
      lowerPen();
      drawLine(font_size/(2*cos((float)25*(2*M_PI/360))), true, 1.0);
      rotate(50);
      lowerPen();
      drawLine(font_size/(2*cos((float)25*(2*M_PI/360))), false, 0.0);
      rotate(65);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'l':
    case 'L':
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      liftPen();
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'm':
    case 'M':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'n':
    case 'N':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(165);
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;

    case '0':
    case 'o':
    case 'O':////////////////////////////
    return;
    
    case 'p':
    case 'P':////////////////////////////
    return;
    
    case 'q':
    case 'Q':////////////////////////////////////////////////////////
    return;
    
    case 'r':
    case 'R':////////////////////////////
    return;
    
    case 's':
    case 'S':////////////////////////////////////////////////////////
    return;
    
    case 't':
    case 'T':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(90);
      lowerPen();
      drawLine(font_size/2, true, 1.0);
      lowerPen();
      drawLine(font_size/2, false, 0.0);
      rotate(270);
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'u':
    case 'U':////////////////////////////
    return;
    
    case 'v':
    case 'V':
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(195);
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90); 
    return;
    
    case 'w':
    case 'W':
      drawLine(font_size, false, 0.0);
      rotate(195);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/(3*cos((float)15*(2*M_PI/360))),false, 0.0);
      rotate(210);
      lowerPen();
      drawLine(font_size/(3*cos((float)15*(2*M_PI/360))),false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos((float)15*(2*M_PI/360)), false, 0.0);
      rotate(195);
      drawLine(font_size, false, 0.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90); 
    return;
    
    case 'x':
    case 'X':
      rotate(340);
      lowerPen();
      drawLine(font_size/cos((float)20*(2*M_PI/360)), false, 0.0);
      rotate(110);
      drawLine((cos((float)70*(2*M_PI/360))*font_size)/cos((float)20*(2*M_PI/360)), false, 0.0);
      rotate(110);
      lowerPen();
      drawLine(font_size/cos((float)20*(2*M_PI/360)), false, 0.0);
      rotate(70);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'y':
    case 'Y':
    return;
    
    case 'z':
    case 'Z':
    return;
    
    case '2':////////////////////////////
    return;
    
    case '3':////////////////////////////
    return;
    
    case '4':
    return;
    
    case '5':////////////////////////////////////////////////////////
    return;
    
    case '6':////////////////////////////
    return;
    
    case '7':
    return;
    
    case '8':////////////////////////////////////////////////////////
    return;
    
    case '9':////////////////////////////
    return;
    
  }
}

void rotate(float degree){
  clearDisplay();
  writeLCDString("Rotate:" + (String)degree);
  delay(1000);
  if(!pen_up)
    liftPen();
  //rotate "degrees" degrees counter-clockwise
  setMotorDirection(RIGHT_MOTOR, BACKWARD);
  analogWrite(Motors[RIGHT_MOTOR].enPin,120);
  analogWrite(Motors[LEFT_MOTOR].enPin,120);
  delay(spin_delay * (degree/360));

  analogWrite(Motors[LEFT_MOTOR].enPin,0);  
  analogWrite(Motors[RIGHT_MOTOR].enPin,0);
  setMotorDirection(RIGHT_MOTOR, FORWARD);
  delay(100);
}

//length is how long of a line to draw
//backtrack is whether or not the robot will return to a point along the line it just drew
//fraction is how far along the line the robot just drew to backtrack, 0.0 <= fraction <= 1.0
//THIS FUNCTION IS NOT YET OPERATIONAL
void drawLine(float line_length, bool backtrack_true, float fraction){
  clearDisplay();
  writeLCDString("drawline: " + (String)line_length);
  delay(500);
  analogWrite(Motors[LEFT_MOTOR].enPin,120);
  analogWrite(Motors[RIGHT_MOTOR].enPin,120);
  delay(1000*line_length);
  analogWrite(Motors[LEFT_MOTOR].enPin,0);
  analogWrite(Motors[RIGHT_MOTOR].enPin,0);
  delay(500);
  prev_length = line_length;
  if(backtrack_true)
    backtrack(fraction);
}
void backtrack(float fraction){
    clearDisplay();
    writeLCDString("backtrack");
    delay(500);
    if(!pen_up)
      liftPen();
    rotate(180);
    drawLine(prev_length*fraction,false,0); //go back to fractional point
}

void liftPen(){ //set pen_up to true
  penservo.write(0);
  pen_up = true;
}
void lowerPen(){// set pen_up to false
  penservo.write(38);
  pen_up = false;
}
