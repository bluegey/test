#ifndef __POWERSUPPLY_TASK_H_
#define __POWERSUPPLY_TASK_H_


//超级电容
typedef enum
{
	 SuperCapacitor_OFF = 0,//电池开，电容关，充电关
   SuperCapacitor_ON = 1,//电池关，电容开，充电关
	 SuperCapacitor_Charge = 2,//电池开，电容关，充电开
}SuperCapacitor_State_e;


extern int16_t SuperCapacitorState;

extern SuperCapacitor_State_e SuperCapacitor_State;

void PowerSupply_Task(void);

#endif

