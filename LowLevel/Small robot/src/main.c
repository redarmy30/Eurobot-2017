/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f4xx_conf.h"

#include "stm32f4xx.h"
#include "Board.h"  //файл инициализации

#include "gpio.h" // работа с портами ввода-вывода
#include "Pins.h" // определение ножек на плате
#include "Interrupts.h"
#include "regulator.h"  // регул€торы колес, кинематика, траекторный

#include "usart.h" //обмен с измерительной тележкой
#include "robot.h"  //определение конфигурации робота и его основных функций
#include "Manipulators.h"
// обмен с компьютером
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char mode;

int main(void)
{
    __disable_irq();
   initAll();


      USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#else
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

       //сброс координат измерительной тележки в случае перезапуска контроллера


//    __disable_irq();
//    conf_pin(EXTI2_PIN, INPUT, PUSH_PULL, FAST_S, PULL_UP);
__enable_irq();

     //   char * str ="mobile robot V1.0";
    //char ch = 5;
    //float duty = 0.09;
    //float duty1 = 0.07;// закрыто (ѕ–»“я√»¬ј≈ћ)
    //float duty2 = 0.024;// открыто(ќ“—ќ≈ƒ»Ќя≈ћ)
//int  test =  0;


/*InPackStruct test;
test.command=0x25;
char robotCoord1[3] = {0.1,0.,0};
test.param = *robotCoord1;*/
//265
//145
//pathPointStr robotCoord1 = {0.0, 0.0, 3.14, NULL,NULL,0,stopVel,stopRot,0,1};
//float robotCoord1[3] = {0.1,0.,0};
float temp[3] ={0.25,-0.25,0};
float anlge =270;
float torka = 1000;
uint16_t detector = 0;
//int ttime = 19000000;
//int ttime1 = 10000000;
//uint16_t  angle = 130;
int angle = 0;
int i, j;
float d;
int p;
/*
pwm goes from 0.6-1.0
*/
while(1){

        setSpeedMaxon(0, 0.5);
       // extiType[10];
        //p = pin_val(EXTI5_PIN);
        /*distance_digital2[0] = pin_val(EXTI7_PIN);
        if (distance_digital2[0]>0) {
                distance_digital2[0] = distance_digital2[0];
        }*/
        //distance_digital2[1] = pin_val(IR_FRONT_RIGHT);
/*        distance_digital2[2] = extiType[6];
        distance_digital2[3] = extiType[7];
        distance_digital2[4] = extiType[8];
        distance_digital2[5] = extiType[9];
*/

    /*for(i=2818;i<100000;i++){
        d = (float)i/100;
        setPWM((char)4, d);
        for(j=0; j<500000; j++);
    }*/
//    for(i=(-100);i<100;i++){
//        d = (float)i/100;
       /* setPWM((char)4, 0.055);
        setPWM((char)4, 0.099);
        setPWM((char)4, 0.107);
        setPWM((char)4, 0.166);*/
//        for(j=0; j<1000000; j++);
//    }

//upCollectorToGetBalls();
//downCollectorWithBalls();

//throwCollectorIntoBox();

  /*  if (pin_val (EXTI2_PIN))
            {   //curState.pidEnabled=1;
                curState.trackEn = 1;}
        else
          {curState.trackEn = 0;
            vTargetGlob[0]=0;
            vTargetGlob[1]=0;
            vTargetGlob[2]=0;

            //curState.pidEnabled=0;
    } */

}
}
