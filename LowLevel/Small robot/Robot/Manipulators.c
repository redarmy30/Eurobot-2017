#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Regulator.h"
#include "Board.h"
#include "math.h"

double timeofred;
char color, color_check[8];

float r,b,R,B,CubesCatcherAngle;
float whole_angle, values[10];

extern int numberofrot;

//values[0] = 0;
//values[1] = 0;
//values[2] = 0;
//values[3] = 0;
//values[4] = 0;
//values[5] = 0;
//values[6] = 0;
//values[7] = 0;
//values[8] = 0;
//values[9] = 0;

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
/*  this commented function is wrong, so don't check it and don't use it
void GetDataForManipulator(void)
{
    int i = 3000000;
    CubesCatcherAngle = adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;//*360/3.3

    while(--i > 0);

    float sum=0;
    for(i=0;i<9;i++){
        values[i] = values[i+1];
        sum+=values[i];
    }
    sum += values[9];
    sum/=10;

    if(( previous_status < 380  &&  previous_status >= 270 ) && ( sum < 90  &&  sum >= 0 ))
         number_full_rotations++;

    if((previous_status >= 0  &&  previous_status < 90 ) && ( sum < 380 && sum >= 270 ))
        number_full_rotations--;

//    if(fabs(CubesCatcherAngle - previous_status)>100)
//        number_full_rotations--;



    values[9] = CubesCatcherAngle;

    whole_angle = number_full_rotations*360 + CubesCatcherAngle;

    previous_status = sum;
}*/

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

void setPositionOfCylinderCarrier(float desiredAngle){

    whole_angle = numberofrot * 360 + adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;
    if(desiredAngle > whole_angle){
 //   setServoToWheelMode(2);
        while(fabs(desiredAngle - whole_angle)>15 && (desiredAngle > whole_angle)){
            setServoMovingSpeed(2, (uint16_t)(600 + 1024), 0x0400);
            setServoMovingSpeed(3, (uint16_t)(600), 0x0000);
            whole_angle = numberofrot * 360 + adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;
        }
    }
    else if(desiredAngle < whole_angle){
 //   setServoToWheelMode(2);
        while(fabs(desiredAngle - whole_angle)>15 && (desiredAngle < whole_angle)){
            setServoMovingSpeed(3, (uint16_t)(600 + 1024), 0x0400);
            setServoMovingSpeed(2, (uint16_t)(600), 0x0000);
            whole_angle = numberofrot * 360 + adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;
        }
    }
    setServoMovingSpeed(2, (uint16_t)0, 0x0000);
    setServoMovingSpeed(3, (uint16_t)0, 0x0000);

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

float CubesCatcherAngle = 0;
float prevCubesCatcherAngle = 0;
float diff;
float arCubesCatcherAngle[10];



float encodermagner(float prevencodermagner){

        arCubesCatcherAngle[9] = adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;//*360/3.3
        int i = 0;
        float smoothed = 0;
        for (i;i<9;i++) {
                arCubesCatcherAngle[i] = arCubesCatcherAngle[i+1];
                smoothed+= arCubesCatcherAngle[i];
        }
        smoothed /= 10;


        if ((arCubesCatcherAngle[0] - prevCubesCatcherAngle) > 100) {
                diff = 1;
                //softDelay(100000);
        }
        else if ((arCubesCatcherAngle[0]- prevCubesCatcherAngle) < -100) {
                diff = -1;
                //softDelay(100000);
        }
        else diff = 0;
        prevCubesCatcherAngle = smoothed;
        if ((diff != 0)  && (prevencodermagner !=0)){
            diff =2;}
        return diff;
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
