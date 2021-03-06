#include "Robot.h"
#include "pins.h"
#include "usart.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "stm32fxxx_it.h"
#include "usbd_cdc_vcp.h"
#include "string.h"
#include "Regulator.h"
#include "interrupts.h"
#include "Board.h"
#include "Communication.h"
#include "manipulators.h"

//float distanceData[3][4] = {0,0,0,0,0,0,0,0,0,0,0,0};
extern char allpointsreached;
float distanceData[3][6] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float distanceFromIR;
bool flag = 1;
uint16_t distance_digital[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t distance_digital1[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t distance_digital2[6];
float vTargetGlob_last[3]={0,0,0};
float robotCoordTarget[3] = {0,0,0}; // Целевые координаты робота в глоб сис-ме координат
float robotSpeedTarget[3] = {0,0,0}; // Целевые скорости робота в глоб сис-ме координат
float motorSpeed[4];                // скорости моторов
float motorCoord[4] = {0,0,0};      // общий пройденный колесом путь
float robotCoord[3] = {0,0,0};       // Координаты робота по показаниям измерительной тележки
float robotSpeed[3] = {0,0,0};       // скорость робота по показаниям измерительной тележки
robStateStruct curState = {1,1,1, 0, 0};    // состояние регуляторов активен-1/неактвен -0
encOutPackStruct outEnc;              //буфер данных отправляемых измерительной тележке

uint32_t * encCnt[4] ={ENCODER1_CNT, ENCODER2_CNT, ENCODER3_CNT, ENCODER4_CNT};  //массив указателей на счетчики энкодеров колес
char  WHEELS[4]= {WHEEL1_CH, WHEEL2_CH, WHEEL3_CH, WHEEL4_CH}; //каналы подкючения колес

//extern CDC_IF_Prop_TypeDef  APP_FOPS;

char execCommand(InPackStruct* cmd) //обработать входящую команду
{
switch(cmd->command)
{
  case 0x01: //Эх
    {
     char *key=  cmd->param;

      if ((key[0] =='E')&&(key[1] =='C')&&(key[2] =='H')&&(key[3] =='O') )
      {
        char * str ="mobile robot V2.0";
        sendAnswer(cmd->command, str, strlen(str)+1);
        }
      }
  break;

  case 0x02:  //Установить текущие координаты
  {
      float *(temp) ={(float*)cmd->param};

      robotCoord[0]= temp[0];
      robotCoord[1]= temp[1];
      robotCoord[2]= temp[2];

      points[0].center[0]= temp[0];
      points[0].center[1]= temp[1];
      points[0].center[2]= temp[2];

      CreatePath(&points[0], &points[0], &curPath);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
}
  break;



  case 0x03: //установить скважность шим
  {
      char  ch = *cmd->param;
      float  temp =*((float*)(cmd->param + 1));
      setPWM( ch - 1, temp);
      char * str ="Ok";
      sendAnswer(cmd->command, str, 3);

  }
  break;

  case 0x04:  //Установить бит направления
  {
      char * ch = cmd->param;
      set_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);

  }
  break;

  case 0x05:  //Снять бит направления
  {
      char * ch = cmd->param;
      reset_pin(PWM_DIR[(*ch)-1]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x06:  //Установить напряжение на двигателе
  {
      char  ch = *cmd->param;
      float duty = *((float*)(cmd->param + 1));
      uint16_t dir = *((uint16_t*)(cmd->param + 2));
      setVoltageMaxon( ch-1, dir, duty);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x08:  //Установить параметры регулятора
  {
      float *(temp) ={(float*)cmd->param};
      char i;
   for (i = 0; i<=3; i++)
  {
        wheelsPidStruct[i].p_k = temp[0];
        wheelsPidStruct[i].i_k = temp[1];
        wheelsPidStruct[i].d_k = temp[2];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x09:  //Установить требуюему скорость двигателей
  {
      float *(temp) ={(float*)cmd->param};
      char i;
      for (i = 0; i<=3; i++)
  {
  	regulatorOut[i] = temp[i];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0B:  //Включить рассчет кинематики
  {

        char * str ="Ok";
        sendAnswer(cmd->command, str, 3);
        softDelay(100000000);
     //   setPositionOfCylinderCarrier(160.0);
      //  goDownWithSuckingManipulator();

   //     sendAnswer(cmd->command, str, 3);

//        switchOnPneumo();
//        softDelay(10000000);
//        servo_rotate_90();
//        goUpWithSuckingManipulator();

//
//        setPositionOfCylinderCarrier(150.0);
//        softDelay(10000000);

//      curState.kinemEn=1;
//      char * str ="Ok";
//      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0C:  //Выключить рассчет кинематики
  {
      curState.kinemEn = 0;
      char * str = "Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0D:  //Задать скорости движения
  {
      float *(temp) ={(float*)cmd->param};
      char i;
   for (i = 0; i<=2; i++)
  {
        vTargetGlob[i] = temp[i];
  }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0E:  //Включить траекторный регулятор
  {
      curState.trackEn=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x0F:  //Выключить траекторный регулятор
  {
      curState.trackEn=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x10:  //очистить очередь точек
  {
      while(lastPoint>0) removePoint(&points[0],&lastPoint);
      points[0].center[0]= robotCoord[0];
      points[0].center[1]= robotCoord[1];
      points[0].center[2]= robotCoord[2];

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x11:  //Добавить точку в очередь
  {

      float *(temp) ={(float*)(cmd->param)};
      char * ch = cmd->param + 12;
      lastPoint++;
      points[lastPoint].center[0] = temp[0];
      points[lastPoint].center[1] = temp[1];
      points[lastPoint].center[2] = temp[2];
      points[lastPoint].speedVelTipe = speedType[*ch];
      points[lastPoint].speedRotTipe = rotType[*(ch)];
      points[lastPoint].endTask = NULL;
      points[lastPoint].movTask = NULL;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x12:  //Состояние очереди точек
  {
      char outdata[15];
      float * temp =(float*)(&outdata[3]);
      char * cntPoint  = (&outdata[0]);
      uint16_t * curPoint =  (uint16_t *)(&outdata[1]);
      *cntPoint= lastPoint;
      *curPoint = totalPointComplite;
      temp[0]= points[0].center[0];
      temp[1]= points[0].center[1];
      temp[2]= points[0].center[2];
      //char * str ="Ok";
      sendAnswer(cmd->command,outdata, 15);
  }
  break;

  case 0x13:  //отправить текущие координаты
  {

      sendAnswer(cmd->command,(char *)robotCoord, sizeof(robotCoord));
  }
  break;

  case 0x14:  //отправить текущую скорость
  {
       if (!pin_val (EXTI2_PIN)){ robotSpeed[0] = 0;
                                  robotSpeed[1] = 0;
                                  robotSpeed[2] = 0;}
      sendAnswer(cmd->command,(char *)robotSpeed, sizeof(robotCoord));
  }
  break;

  case 0x15:  //Задать скорость движения
  {
      float *(temp) ={(float*)(cmd->param)};
      char i;
      for (i = 0; i<=4; i++)
        normalVelFast[i]= temp[i];
      for (i = 0; i<=4; i++)
        stopVelFast[i]= temp[i];
      stopVelFast[2]=-0.2;

      char * str = "Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

   case 0x16:  //установить режим ножки
   {

    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {
    pinType[ch] = *((char *)(cmd->param +1));
      if (pinType[ch] == ADC_ANALOG_PIN )
          conf_pin(GENERAL_PIN[ch], ANALOG, PUSH_PULL, FAST_S, NO_PULL_UP);
      if (pinType[ch] == ADC_DIG_INPUT )
          conf_pin(GENERAL_PIN[ch], INPUT, PUSH_PULL, FAST_S, NO_PULL_UP);
      if (pinType[ch] == ADC_DIG_OUTPUT )
          conf_pin(GENERAL_PIN[ch], GENERAL, PUSH_PULL, FAST_S, NO_PULL_UP);

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
    }
  }
  break;

  case 0x17:  //отправить состояние выбранного входа АЦП
  {
      char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
      sendAnswer(cmd->command,(char *)&(adcData[ch]), sizeof(uint16_t));
  }
  break;

  case 0x18:  //отправить состояние всех АЦП
  {
      sendAnswer(cmd->command,(char *)adcData, sizeof(adcData));
  }
  break;

  case 0x19:  //отправить состояние входа
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {
      char temp =  (pin_val(GENERAL_PIN[ch])!=0);
      sendAnswer(cmd->command,&temp, sizeof(temp));
    }
  }
  break;

  case 0x1A:  //отправить состояние всех входов
  {
      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(GENERAL_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;

  case 0x1B:  //установить состояние выхода
  {
      char ch = (*((char *)(cmd->param))) -1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(GENERAL_PIN[ch]); else
                              set_pin(GENERAL_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x1C:  //отправить текущий режим ножки
  {
    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(pinType[ch]), sizeof(uint8_t));
  }
  break;

  case 0x1D:  //установить режим ножки EXTI
  {

    char ch = (*((char *)(cmd->param))) -1;
    if (ch<10)
    {

      extiType[ch] = (*((char *)(cmd->param +1)));
      if (extiType[ch] == EXTI_BOTH )
      {
          conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch],  EXTI_BOTH_EDGES);
      }

      if (extiType[ch] == EXTI_RISE )
      {
          conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch], EXTI_RISING_EDGE);
      }
      if (extiType[ch] == EXTI_FALL )
      {
           conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, PULL_UP);
          add_ext_interrupt(EXTI_PIN[ch], EXTI_FALLING_EDGE) ;
      }
      if (extiType[ch] == EXTI_DIG_INPUT )
      {
        conf_pin(EXTI_PIN[ch], INPUT, PUSH_PULL, FAST_S, NO_PULL_UP);
        clear_ext_interrupt(EXTI_PIN[ch]) ;
      }

      if (extiType[ch] == EXTI_DIG_OUTPUT )
      {
        conf_pin(EXTI_PIN[ch], GENERAL, PUSH_PULL, FAST_S, NO_PULL_UP);
        clear_ext_interrupt(EXTI_PIN[ch]) ;

      }

      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
    }
  }
  break;

  case 0x1E:  //отправить состояние входа
  {
    char ch = *((char *)(cmd->param))-1;
    if ((ch)<10)
    {
      char temp = ch;
      if (pin_val(EXTI_PIN[ch])) temp |=0x80;
      sendAnswer(cmd->command,&temp, sizeof(temp));
    }
  }
  break;

  case 0x1F:  //отправить состояние всех входов
  {

      char temp[10];
      char i ;
      for ( i = 0; i<10; i++)  temp[i] = (pin_val(EXTI_PIN[i])!=0);
      sendAnswer(cmd->command,(char *)temp, sizeof(temp));
  }
  break;

  case 0x20:  //установить состояние выхода
  {
       char ch = *((char *)(cmd->param))-1;
      if (ch<10)
      if (*(cmd->param+1)==0) reset_pin(EXTI_PIN[ch]); else
                              set_pin(EXTI_PIN[ch]);
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x21:  //отправить текущий режим ножки
  {
    char ch = *((char *)(cmd->param))-1;
    if (ch<10)
    sendAnswer(cmd->command,(char *) &(extiType[ch]), sizeof(uint8_t));
  }
  break;

  case 0x22:  //установить состояние выхода +12В
  {
      char ch = (*((char *)(cmd->param)))-1;
      if (ch<6)
      {

       if (*(cmd->param+1)==0)
            reset_pin(V12_PIN[ch]); else
                              set_pin(V12_PIN[ch]);
      }
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x23:  //Выключить ПИД регуляторы приводов
  {
      curState.pidEnabled=0;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;

  case 0x24:  //Включить ПИД регуляторы приводов
  {
      curState.pidEnabled=1;
      char * str ="Ok";
      sendAnswer(cmd->command,str, 3);
  }
  break;


case 0x25:    // set current coordinate
  {
      float *(temp) ={(float*)cmd->param};

        robotCoord[0] =temp[0];
        robotCoord[1] =temp[1];
        robotCoord[2] =temp[2];

      char * str ="Ok";

      sendAnswer(cmd->command,str, 3);
  }

break;
   case 0x26:  //set dynamixel angle
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(angle) ={(uint16_t*)(cmd->param + 1)};
      if (setServoMovingSpeed(ID, angle))
      {
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x27:  //set CW angle limit
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(limit) ={(uint16_t*)(cmd->param + 1)};
      if (setServoCWAngleLimit(ID, limit))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;



  case 0x28:  //set CCW angle limit
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(limit) ={(uint16_t*)(cmd->param + 1)};
      if (setServoCCWAngleLimit(ID, limit))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x29:  //set servo moving speed
  {
      uint8_t *(ID) ={(uint8_t*)cmd->param};
      uint16_t *(speed) ={(uint16_t*)(cmd->param + 1)};
      uint16_t *(direction) ={(uint16_t*)(cmd->param + 3)};
      if (setServoMovingSpeed(ID, speed, direction))
      {
          char * str ="Ok";
          sendAnswer(cmd->command,str, 3);
      }
  }
  break;

  case 0x2A:  //add a point to the beginning of Queue
  {
      float *(temp) = (float*)(cmd->param);
      char * ch = cmd->param + 12;
      addPointInFrontOfQueue(&points[0], &temp[0], &ch, &lastPoint);
      CreatePath(&points[0], &robotCoord[0], &curPath);

      char * str ="Ok";
      sendAnswer(cmd->command, str, 3);
  }
  break;




  case 0x2D: //pump manipulator movement for EuroBot 2017 (old)
  {
        servo_elevate_out(); // move servo out
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
    break;

    case 0x2E: //pump manipulator movement for EuroBot 2017 (old)
    {
        servo_elevate_in(); // move servo inside
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
    break;



case 0x32:  // Flag of reached point
  {
      sendAnswer(cmd->command, (char *)&allpointsreached, sizeof(allpointsreached));
  }
  break;

case 0x37:   //open umbrella
{
    set_pin (PIN6_12V); // крутимся
    softDelay(10000000);
    softDelay(12000000);
    softDelay( 000000);

    reset_pin (PIN6_12V); // не крутимся

    char * str ="Ok";
    sendAnswer(cmd->command, str, 3);
}

break;
case 0x36:
    {
    if (pin_val (EXTI2_PIN))
    {   char * str ="1";
        sendAnswer(cmd->command, str, 2);
    }
    else
    {
        char * str ="0";
        sendAnswer(cmd->command, str, 2);
    }


    }
break;

  case 0x40:  // Stop command
  {
    curState.pidEnabled = 0;
    char i;
    for (i = 0; i < 4; i++)
    {
        setVoltageMaxon(WHEELS[i], (uint8_t) 1,  (float) 0);
    }
    char * str ="Ok";
    sendAnswer(cmd->command,str, 3);
  }
  break;

case 0x38:        // avoidance ENABLED
{       curState.collisionAvEn   = 1;
        char * str ="Ok";
       sendAnswer(cmd->command, str, 3);

}
break;

case 0x39:        // avoidance ENABLED
{       curState.collisionAvEn = 0;
        char * str ="Ok";
       sendAnswer(cmd->command, str, 3);
}
break;

case 0x3A: // Distance from ultrasonic sensors
  {

        distance_digital2[0] = pin_val(IR_FRONT_LEFT);
        distance_digital2[1] = pin_val(IR_FRONT_RIGHT);
        distance_digital2[2] = pin_val(IR_FRONT_TOP);
        distance_digital2[3] = pin_val(IR_BACK);

        sendAnswer(cmd->command, (char* )distance_digital2, sizeof(distance_digital2));

  }
   break;

case 0x3D: // RGB sensor for cylinder EuroBot 2017
    {
        char *color = getCurrentColor();
        sendAnswer(cmd->command, color, 2);
    }
    break;


   case 0x3E: // switch on pump
    {
        switchOnPneumo();
        char * str ="Ok";
        sendAnswer(cmd->command, str, 3);
    }
    break;

    case 0x3F: // switch off pump
    {
        switchOffPneumo();
        char * str ="Ok";
        sendAnswer(cmd->command, str, 3);
    }
    break;

    case 0x41:  //pump manipulator rotation for EuroBot 2017
    {
        servo_rotate_90(); // rotate the pump 90 degrees - horizontal
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
    break;

    case 0x42:  //pump manipulator rotation for EuroBot 2017
    {
        servo_rotate_180(); // rotate the pump 180 degrees - vertical
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
    break;

    case 0x4A: // Sucking manipulator
    {
        goInsideWithSuckingManipulator();
    }
    break;

    case 0x44:                //this function still needs to be tested, so DON'T CALL IT YET DENIS
    {
        increaseByGivenAngle(LIFT_CYLINDER);
        //it is for holding it
    }
    break;

    case 0x45:                //this function still needs to be tested, so DON'T CALL IT YET DENIS
    {
        increaseByGivenAngle(STORE_CYLINDER);
        //it is for lifting storing it
    }
    break;

    case 0x46:                //this function still needs to be tested, so DON'T CALL IT YET DENIS
    {
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        servo_rotate_180();
    }
    break;

    case 0x47:                //this function still needs to be tested, so DON'T CALL IT YET DENIS
    {
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        switchOffPneumo();
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();
    }
    break;

    case 0x48:                //this function still needs to be tested, so DON'T CALL IT YET DENIS
    {
        decreaseByGivenAngle(LIFT_CYLINDER + STORE_CYLINDER);
    }
    break;

    case 0x49: // Sucking manipulator
    {
        goOutsideWithSuckingManipulator();
    }
    break;

    case 0x4B: // Sucking manipulator
    {
        dropAllCylinders();
    }
    break;

case 0x43: // Generate new trajectory with correction
{
    float *(temp) ={(float*)cmd->param};
    char * ch = cmd->param + 24;
    robotCoord[0] = temp[0]; //# TODO TEST SPEED
    robotCoord[1] = temp[1];
    robotCoord[2] = temp[2];
    lastPoint++;
    points[lastPoint].center[0] = temp[3];
    points[lastPoint].center[1] = temp[4];
    points[lastPoint].center[2] = temp[5];
    points[lastPoint].speedVelTipe = speedType[*ch];
    points[lastPoint].speedRotTipe = rotType[*ch];
    points[lastPoint].endTask = NULL;
    points[lastPoint].movTask = NULL;
    char * str ="Ok";
    sendAnswer(cmd->command, str, 3);
}
    break;
         /*
case 0x2E:
{
    SERVO_EVEVATE_IN(); // move servo inside
    char * str ="Ok";
    sendAnswer(cmd->command,str, 3);

}
break;

case 0x45:
    {
        SERVO_EVEVATE_OUT(); // move servo out
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
break;

case 0x46:
    {
        servo_ROTATE_90(); // rotate the pump 90 degrees
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
break;

case 0x47:
    {
        servo_ROTATE_180(); // rotate the pump 180 degrees
        char * str ="Ok";
        sendAnswer(cmd->command,str, 3);
    }
break;

*/






    default:
    return 0;
  break;
}



}



/*void checkCollisionAvoid_small(float * rV, float* vTargetGlob)
{

          float realRad        = robotCoord[2];

  //float Ml[4][2]     = {(sinus+cosinus), (cosinus-sinus), (cosinus-sinus), -(sinus+cosinus), (cosinus-sinus), -(sinus+cosinus), (sinus+cosinus), (cosinus-sinus)};
 // float Mfi[4]       = {-(0.14), -(0.14),-(0.14), -(0.14)};  //матрица расчета угловой скорости
            float Mrot[3][3]   = {cos(realRad) , sin(realRad), 0,
                        -sin(realRad), cos(realRad), 0,
                                    0,            0, 1};  //матрица пересчета глобальных скоростей в локальные
            float localVelocity[3];
            matrixMultiplyM2M(&Mrot[0][0], 3, 3, vTargetGlob, 3, 1, &localVelocity[0]);//Ml*Velocity speed in local coordinate system

    //if (*flag) {memcpy( vTargetGlob_last, vTargetGlob, sizeof( float)*3);}
//ang3 = adcData[3] * 0.0822 * 2.54;
    //if (fabs( vTargetGlob[2])>0.05 &&((distanceData[1][5]<=threshhold ) || (distanceData[2][5]<=threshhold )||distanceData[0][5]<=threshhold))
     //   {stopmove();}
    //if ((vTargetGlob[0] >0) && (vTargetGlob[1]>0) && (distanceData[1][5]<=threshhold )){
        //stopmove();
      //  stopmove();
       // }

     if ((localVelocity[0]>0) && (localVelocity[1]<0) && ((distanceData[0][5]<=threshhold ) || (distanceData[1][5]<=threshhold ) || (distance_digital[0] == 0))){
            stopmove();}

    else if ((localVelocity[0]<=0) && (localVelocity[1]<=0) && (distanceData[0][5]<=threshhold || (distance_digital[0] == 0))  ) {
        stopmove();}
//    else if ((localVelocity[0]<=0) && (localVelocity[1] >0) && ((distanceData[2][5]<=threshhold ) || (distance_digital1[0] == 0))) {
  //      stopmove();}
    else {
        //curState.trackEn = 1;
        //*flag = 1;
        }
    //*flag = 0;
}


void stopmove(){
        //curState.trackEn=0;
        vTargetGlob[2]=0.0;
        vTargetGlob[0]=0.0;
        vTargetGlob[1]=0.0;
        vTargetGlobF[2]=0.0;
        vTargetGlobF[0]=0.0;
        vTargetGlobF[1]=0.0;
        }

//void takeadc(int adc_number1,int adc_number2,int adc_number3)
//{

    //figure out how many distance sensors we will have - 6 IR



/*
    for(i=1;i<8:i++)distance_digital[i]=distance_digital[i+1];
    distance_digital[8] = pin_val(EXTI1_PIN);

    //distance_digital[0]= distance_digital[1]*distance_digital[2]*distance_digital[3]*distance_digital[4]*distance_digital[5]*distance_digital[6];
    int i =0;
    for (i = 1; i <= 5; i++) // 0-4
    {
        if (distance_digital[i]==0)
        {
            distance_digital[9]=1; //flag
        }
    }
    if (distance_digital[9]==1)
    {distance_digital[0]=0;
     distance_digital[9]=0;}
    else
     {distance_digital[0]=1;
     distance_digital[9]=0;
     }

    for(i=1;i<8:i++)distance_digital1[i]=distance_digital1[i+1];
    distance_digital1[8] = pin_val(EXTI2_PIN);

    //distance_digital[0]= distance_digital[1]*distance_digital[2]*distance_digital[3]*distance_digital[4]*distance_digital[5]*distance_digital[6];

    for (i = 1; i <= 5; i++) // 0-4
    {
        if (distance_digital1[i]==0)
        {
            distance_digital1[9]=1; //flag
        }
    }
    if (distance_digital1[9]==1)
    {distance_digital1[0]=0; //this means there is an obstacle in this sensor
     distance_digital1[9]=0;}
    else
     {distance_digital1[0]=1;
     distance_digital1[9]=0;
     }
*/

//}

/*void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks-- );
}
    uint16_t stVal = 0;
    uint16_t finalVal = 0;
    uint16_t curLoad;*/
//______________________________________________________//


