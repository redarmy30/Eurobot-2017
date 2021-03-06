/*

Upper level functions for communication with Odroid board.

*/

#include "Communication.h"
#include "stdint.h"
#include "robot.h"
InPackStruct inCommand ={0xFA, 0xAF, 0x00, 0x00, &param[0]}; //структура входящего пакета
void pushByte(char inByte) // поиск, формирование и проверка входящего пакета в потоке данных
{
  char j;
  uint16_t checkSum;
  uint16_t * test;
  inData[dataIndex++] = inByte;

  if((inData[0] == SYNC_BYTE) && (inData[1] == ADR_BYTE))  //поиск заголовка
  {
    if( (dataIndex >= inData[2]) && (dataIndex > 3) ) //проверка длинны пакета
    {
      checkSum = packetCheck(&inData[0], inData[2] - CHECK_SIZE);
      test = ( uint16_t *) &inData[inData[2] - CHECK_SIZE];
      if (*test == checkSum) // проверка CRC
      {
        inCommand.packLen = inData[2];
        for (j=0; j < inCommand.packLen - CHECK_SIZE - HEADER_SIZE; j++)  //Копирование параметров
                      *(inCommand.param + j) = inData[4 + j];
        inCommand.command = inData[3];
        execCommand(&inCommand);     //выполнение команды
      }
      dataIndex = 0;
      inData[0] = 0;
      inData[1] = 0;
    }
  }
  else
  {
    if (dataIndex > 1)
    {
      inData[0] = inData[1];
      inData[1] = 0;
      dataIndex = 1;
    }
  }
}

extern uint8_t  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                     These data will be sent over USB IN endpoint
                                     in the CDC core functions. */
extern uint32_t APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                     start address when writing received data
                                     in the buffer APP_Rx_Buffer. */

char sendAnswer(char cmd, char * param, int paramSize) // отправить ответ по USB
{

         __disable_irq();
         outData[0] = 0xFA;
         outData[1] = 0xFA;
         outData[2] = paramSize + HEADER_SIZE + CHECK_SIZE;
         outData[3] = cmd;
         memcpy(&outData[4], param, paramSize);

         *((int16_t*)&outData[paramSize + HEADER_SIZE]) = (int16_t) packetCheck(&outData[0], paramSize + HEADER_SIZE);
         int _size = paramSize + HEADER_SIZE + CHECK_SIZE  ;
         int i;
         for (i=0; i < _size; i++) putchar(outData[i]);

         if (APP_Rx_ptr_in + _size < APP_RX_DATA_SIZE)
         {
            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], outData, _size);
            APP_Rx_ptr_in += _size;
         }
         else
         {
            int freeSpace = APP_RX_DATA_SIZE - APP_Rx_ptr_in;

            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], outData, freeSpace);
            APP_Rx_ptr_in = 0;
            memcpy(&APP_Rx_Buffer[APP_Rx_ptr_in], &outData[freeSpace], _size - freeSpace);
            APP_Rx_ptr_in += _size - freeSpace;
         }
         //     APP_FOPS.pIf_DataTx((uint8_t*)outData,
         //             paramSize+HEADER_SIZE+CHECK_SIZE);

          __enable_irq();
         return paramSize + HEADER_SIZE + CHECK_SIZE;
}


