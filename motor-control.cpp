#include <wiringPi.h>
#include <iostream>

#define pwm 4 // pin7
#define IN1 27// pin13
#define IN2 17//pin11

using namespace std;

int main(){
 wiringPiSetup () ;

pinMode(pwm, OUTPUT) ;
pinMode(IN1, OUTPUT) ;
pinMode(IN2, OUTPUT) ;
 while (1)
 {
    cout<< "THIS HAPPENED0";
digitalWrite(pwm, HIGH);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW) ;
delay(1000);


digitalWrite(pwm, HIGH);
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH) ;
delay(1000);


 }



return 0 ;
}