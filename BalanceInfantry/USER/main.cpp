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
	//�����ʼ��
	BSP_Init();
	delay_ms(2500);
	//��ʼ����������
	StartTask();
	while (1)
		;
}
