#ifndef colorcfg_h
#define colorcfg_h

#include "color.h"


/*����ɫ���ѯ�ķ�Χ  ͼ����LCD������*/
#define IMG_X 40			      		//ͼƬx����120
#define IMG_Y 40               //ͼƬy����240 
#define IMG_W 200             	//ͼƬ���240
#define IMG_H 280            	 	//ͼƬ�߶�320

#define ALLOW_FAIL_PER       2   

#define COLOR_RANG           30    //�趨��ɫ��ƫ�Ʒ�Χ Խ��Խ����ʶ�� ̫��������ʶ��
#define COLOR_NUM            7     //�趨׷����ɫ����Ŀ
#define OBJECT_NUM          1     //�趨׷����������
extern u8 global_page;									//��ǰ��ɫ��
extern u8 color_list[COLOR_NUM][7+7];		//��ɫ�ַ�
extern SEARCH_AREA_t area[OBJECT_NUM+1];							//������������
extern RESULT_t result[OBJECT_NUM];			//�����������
extern TARGET_CONDITION_t condition[COLOR_NUM];//����Ŀ�����
extern Quadrant		Quadrant_control;

#define leftup      0//����
#define leftdown    1//����
#define rightup     2//����
#define rightdown   3//����
#define Distinguish_Method    1
#define LCD_READPOINT( usX, usY )  LCD_ReadPoint(usX,usY)//������㺯��
void Enlarge_or_reduce(u16 wide,u16 height);
#endif

