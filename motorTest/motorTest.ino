//Arduino PWM Speed Control：
int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;                        
    int i;
void setup() 
{ 
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT); 
} 

void loop() 
{ 

    digitalWrite(M1,LOW);   
    digitalWrite(M2, HIGH);   
    for(i=5; i<=255; i+=5){    
    analogWrite(E1, i);   //PWM Speed Control
    analogWrite(E2, i);   //PWM Speed Control
    delay(300);
    }
    if(i>=255){
    for(i=255; i>=5; i-=5){    
    analogWrite(E1, i);   //PWM Speed Control
    analogWrite(E2, i);   //PWM Speed Control
    delay(300);
    }
    }

}
