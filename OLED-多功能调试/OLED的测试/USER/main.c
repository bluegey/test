#include "main.h"

int main()
{
	Module_Init();//先初始化外设
	StartTask_Init();//外设初始化后进入任务
	while(1);
}

