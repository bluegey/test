#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "sys.h"

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;

} RC_ctrl_t;
const RC_ctrl_t *get_RC_DATD(void);
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#endif

