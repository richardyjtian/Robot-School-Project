#include <Servo.h>

Servo myservo;
myservo.attach(SERVO_PEN);
myservo.write(0);
delay(500);
  
//integer > 10
int font_size = 4;

//integer > 0
int space = 2;

bool pen_up;

int prev_length;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

//***THIS FUNCTION IS NOT YET OPERATIONAL
void drawWord(char *word){
  if(strlen(word) > 1){
    for(int i = 0; i < strlen(word); i++){
      drawWord(word[i]);
    }  
    return;
  }
  
  switch(word[i]){

    case 'a':
    case 'A':
      int A_hypotenuse = font_size/cos(15);
      rotate(345);
      lowerPen();
      drawLine(A_hypotenuse, true, 1/2);
      rotate(105);
      lowerPen();
      drawLine(A_hypotenuse*cos(75), false, 0.0);
      rotate(105);
      lowerPen();
      drawLine(A_hypotenuse/2, true, 1.0);
      lowerPen();
      drawLine(A_hypotenuse/2, false, 0.0);
      rotate(75);
      drawLine(space, false, 0.0);
      rotate(90);
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
      drawLine(font_size/(2*cos(25)), true, 1.0);
      rotate(50);
      lowerPen();
      drawLine(font_size/(2*cos(25)), false, 0.0);
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
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
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
      drawLine(font_size/cos(15), false, 0.0);
      rotate(165);
      lowerPen();
      drawLine(font_size, true, 1.0);
      rotate(90);
      drawLine(space, false, 0.0);
      rotate(90);
    return;

    case '0':
    case 'o':
    case 'O':
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawline(font_size/2, false, 0.0);
      rotate(270);
      lowerPen();
      drawLine(font_size, false, 0.0);
      rotate(270);
      lowerPen();
      drawline(font_size/2, true, 1.0);
      drawLine(space, false, 0.0);
      rotate(90);
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
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
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
      drawLine(font_size/cos(15), false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/(3*cos(15)),false, 0.0);
      rotate(210);
      lowerPen();
      drawLine(font_size/(3*cos(15)),false, 0.0);
      rotate(150);
      lowerPen();
      drawLine(font_size/cos(15), false, 0.0);
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
      drawLine(font_size/cos(20), false, 0.0);
      rotate(110);
      drawLine((cos(70)*font_size)/cos(20), false, 0.0);
      rotate(110);
      lowerPen();
      drawLine(font_size/cos(20), false, 0.0);
      rotate(70);
      drawLine(space, false, 0.0);
      rotate(90);
    return;
    
    case 'y':////////////////////////////
    case 'Y':
    return;
    
    case 'z':
    case 'Z':
      drawLine(font_size, false, 0.0);
      rotate(270);
      drawLine,(font_size/2, false, 0.0);
      rotate(200);
      drawLine(font_size/(2*cos(20));
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

  //***THIS FUNCTION IS NOT YET OPERATIONAL
  void rotate(int degrees){
    int num = degrees/90;
    if(!pen_up)
      liftPen();
    //rotate "degrees" degrees counter-clockwise
    for(int i = 1; i <= num; i++){
      keepAlign(160, right_forward, left_backward);
    }
    align();
  }

  //length is how long of a line to draw
  //backtrack is whether or not the robot will return to a point along the line it just drew
  //fraction is how far along the line the robot just drew to backtrack, 0.0 <= fraction <= 1.0
  //***THIS FUNCTION IS NOT YET OPERATIONAL
  void drawLine(int length, bool backtrack, float fraction){
    for(int i = 1; i <= length; i++){
      keepAlign(255, right_forward, left_forward);  
    }
    align();
    prev_length = length;
    
    if(backtrack)
      backtrack(fraction);
  }
  
  void backtrack(float fraction){
      if(!pen_up)
        liftPen();
      rotate(180);
      drawLine(prev_length*fraction, false, 0.0);
  }

  void liftPen(){
    myServo.write(20);
    pen_up = true;
  }

  void lowerPen(){
    myServo.write(0);
    pen_up = false;
  }
  
}
