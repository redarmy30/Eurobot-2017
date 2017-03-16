#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Regulator.h"
#include "Board.h"


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
    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
    set_pin(PIN3_12V);
    set_pin(PIN4_12V);
    set_pin(PIN5_12V);
    set_pin(PIN6_12V);
    return 0;
}

bool switchOffPneumo()
{
    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
    reset_pin(PIN3_12V);
    reset_pin(PIN4_12V);
    reset_pin(PIN5_12V);
    reset_pin(PIN6_12V);
    return 0;
}


///////////////////////////////////////////////////////////////
