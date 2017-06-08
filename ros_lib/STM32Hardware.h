#ifndef _STM32Hardware_H
#define  _STM32Hardware_H

//#include "config/stm32plus.h"
//#include "utils/UsartWithBuffer.h"
#include "usbd_cdc_if.h"
#include "main.h"
extern __int64 MILLISEONS;
extern USBD_HandleTypeDef hUsbDeviceFS;

class STM32Hardware {


  public:
//  STM32Hardware(SERIAL_CLASS* _com , long baud= 57600){
//      com = _com;
//      baud_ = baud;
//    }
    STM32Hardware()
    {
//      com = new SERIAL_CLASS(57600);
      baud_ = 57600;
    }
    STM32Hardware(STM32Hardware& h){
      this->baud_ = h.baud_;
    }

    void setBaud(long baud){
      this->baud_= baud;
    }

    int getBaud(){return baud_;}

    void init(){
      //iostream->begin(baud_);
    }

    int read(){
       return getch();  //define in usbd_cdc_if.c
			
    };

    void write(uint8_t* data, int length)
		{
			CDC_Transmit_FS(data,length);
    }

    unsigned long time(){return MILLISEONS;}

  protected:
    int* com;
    long baud_;
};
#endif
