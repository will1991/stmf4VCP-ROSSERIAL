#ifndef _STM32Hardware_H
#define  _STM32Hardware_H

//#include "config/stm32plus.h"
//#include "utils/UsartWithBuffer.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "usb_device.h"
//extern __int64 MILLISEONS;
extern __IO uint32_t uwTick;
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
       MX_USB_DEVICE_Init();//iostream->begin(baud_);
    }

    int read(){
       return getch();  //define in usbd_cdc_if.c
			
    };

    void write(uint8_t* data, int length)
		{
			while(CDC_Transmit_FS(data,length)!= USBD_OK);
    }

    unsigned long time(){return uwTick;}

  protected:
    int* com;
    long baud_;
};
#endif
