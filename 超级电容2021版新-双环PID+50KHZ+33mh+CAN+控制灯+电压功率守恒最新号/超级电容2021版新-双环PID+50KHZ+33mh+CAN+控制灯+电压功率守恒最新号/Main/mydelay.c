#include "stm32f10x.h"
#include "mydelay.h"

void delay_ms(u16 time)
{
   u16 i=0;
   while(time--)
   {
      i=12000;  //自己定义
      while(i--);
   }
}

void delay_us(u16 time)
{
   u16 i=0;
   while(time--)
   {
      i=10;//自己定义
      while(i--);
   }
}
