#include "Start.h"
//#include <iostream>
// using namespace std;

//#if 1
//#ifdef __cplusplus
//   extern "C" {
//#endif
// extern void $Super$$main(void);
// void $Sub$$main(void)  {
//  uart_init(115200);
//  $Super$$main();
//}
//#ifdef __cplusplus
//    } //extern "C"
//#endif
//		#endif//

int main(void)
{
	//外设初始化
	BSP_Init();
	delay_ms(2500);
	//开始任务任务函数
	StartTask();
	while (1)
		;
}
