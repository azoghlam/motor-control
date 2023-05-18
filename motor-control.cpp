#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>

#define pwm 7 // pin7
#define IN1 2// pin13 gpio27
#define IN2 0//pin11 gpio 17





// using namespace std;

int main(){

    std::cout << "program started" <<std::endl;

    wiringPiSetup () ;

    std::cout << "wiring pi setup done"<<std::endl;

    pinMode(pwm, OUTPUT) ;
    pinMode(IN1, OUTPUT) ;
    pinMode(IN2, OUTPUT) ;
    softPwmCreate(pwm, 0, 255);
    while (1)
    {

        
       
       // digitalWrite(pwm, HIGH);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW) ;
        for(int x;x<256;x++){
            softPwmWrite(pwm, x);
            std::cout<< x <<std::endl;
        }
        delay(1000);
        // softPwmWrite(pwm, 100);
        // digitalWrite(IN1, HIGH);
        // digitalWrite(IN2, LOW) ;
        // delay(1000);
        // softPwmWrite(pwm, 255);
        // digitalWrite(IN1, HIGH);
        // digitalWrite(IN2, LOW) ;
    // digitalWrite(IN1, LOW);
      //  digitalWrite(IN2, HIGH) ;
       // delay(1000);
        


    }

    return 0;
}