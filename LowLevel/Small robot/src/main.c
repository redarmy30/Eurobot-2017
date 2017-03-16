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
#include "Board.h"

#include "gpio.h"
#include "Pins.h"
#include "Interrupts.h"
#include "Regulator.h"
#include "usart.h"
#include "Robot.h"
#include "Manipulators.h"
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

    __enable_irq();

    while(1)
    {

    }
}
