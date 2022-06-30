#ifndef __CAN_H
#define __CAN_H
#ifdef __cplusplus
extern "C"
{
#endif
#include "sys.h"

    u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode); // CAN锟斤拷始锟斤拷
    u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
    // void CAN_SetMsg(int32_t out1,int32_t out2,int32_t out3,int32_t out4);
#ifdef __cplusplus
}
#endif
#endif
