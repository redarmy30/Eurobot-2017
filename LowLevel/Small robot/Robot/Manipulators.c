#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Regulator.h"
#include "Board.h"

extern double timeofred;
extern char color, color_check[8];
extern float r,b,R,B;

void softDelay(__IO unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}


bool goUpWithSuckingManipulator(){

    set_pin(INPUT1_CONTROL);
    reset_pin(INPUT2_CONTROL);

    while(!pin_val(UPPER_SWITCH));


    reset_pin(INPUT1_CONTROL);
}

bool goDownWithSuckingManipulator(){

    set_pin(INPUT2_CONTROL);
    reset_pin(INPUT1_CONTROL);


    while(!pin_val(DOWN_SWITCH));



    reset_pin(INPUT2_CONTROL);
}

char getColor(){


}

char getCurrentColor(){
    int z = 1000;
    while(z>0){
        z--;
        reset_pin(EXTI1_PIN);//red
        int j,i = 15000;
            for(; i> 0; i--);
        r = timeofred;
        R = 10000./(r);
        i = 15000;
            for(; i> 0; i--);

        set_pin(EXTI1_PIN);//blue

        i = 15000;
            for(; i> 0; i--);
        b = timeofred;
        B = 10000./(b);
////
        if(R >= B){
            if(R/B>1.5)color = 'Y';
            else color = 'W';
        }
        else{
            if(B<100 && B/R>1.1) color = 'B';
            else color = 'W';
        }
        softDelay(500);
        for(j=0;j<7;j++)color_check[j] = color_check[j+1]; //filter
        color_check[7] = color;

        for(j=0;j<8;j++)if(color_check[i] != color){color='n';break;}

        if(color!='n') break;
    }

    return color;
}

void servo_elevate_in()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_IN);
}

void servo_elevate_out()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_OUT);
}

void servo_rotate_90()
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) SERVO_ROTATE_90);
}

void servo_rotate_180()
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) SERVO_ROTATE_180);
}

bool switchOnPneumo()
{
//    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
//    set_pin(PIN3_12V);
//    set_pin(PIN4_12V);
//    set_pin(PIN5_12V);
//    set_pin(PIN6_12V);
    return 0;
}

bool switchOffPneumo()
{
//    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
//    reset_pin(PIN3_12V);
//    reset_pin(PIN4_12V);
//    reset_pin(PIN5_12V);
//    reset_pin(PIN6_12V);
    return 0;
}


///////////////////////////////////////////////////////////////
