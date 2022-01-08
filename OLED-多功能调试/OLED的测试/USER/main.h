/*=========sys=========*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
/*========FreeRtos=====*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
/*=========TASK========*/
#include "Start_Task.h"
#include "Motor_Task.h"
#include "OLED_Task.h"
#include "Can_Receive.h"
/*========HARDWARE=====*/
#include "led.h"
#include "key.h"
#include "oled.h"
#include "Exit.h"
#include "can.h"
#include "remote.h"
