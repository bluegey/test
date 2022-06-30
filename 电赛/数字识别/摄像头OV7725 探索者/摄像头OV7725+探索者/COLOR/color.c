#include "colorcfg.h"
#include "sys.h"
#include "lcd.h"
#include <math.h>
#include <stdio.h>
#include "delay.h"
#include <stdbool.h>
#include "sdio_sdcard.h"
int x1, y1, x2, y2, x3, y3, kk;
RESULT_t result[OBJECT_NUM];
Quadrant Quadrant_control;
uint16_t shapdis[6] = {0, 0, 0, 0, 0, 0};
u8 global_page = 0;
u8 object_flag = 3;
#define Minimum_step 5
#define fail_flag 5
#define DEBUG 1
#define WINDOW_WIDTH 240
u8 color_list[COLOR_NUM][7 + 7] = {
	{"Red          "},
	{"Origin       "},
	{"Blue         "},
	{"Greed        "},
	{"White        "},
	{"Yewllo       "},
	{"s"}};
TARGET_CONDITION_t condition[COLOR_NUM] = {
	{
		0,	// 200,//20,			      //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		10, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		//��Ҫ���� ����������Ͷȷ�Χ�ڡ� �����塣
		0,	// 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		10, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		//��Ҫ���� �����������(�Ҷ�)��Χ�ڡ� �����塣
		0,	// 0,//50,     			//Ŀ����С���ȣ�L_MIN
		10, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		//��Ҫ���� ���������С��Χ�ڡ� �����塣
		35, //Ŀ����С��ȣ�WIDTH_MIN
		60, //Ŀ����С�߶ȣ�HEIGHT_MIN

		160, //Ŀ������ȣ�WIDTH_MAX
		260, //Ŀ�����߶ȣ�HEIGHT_MAX
	},
	/*------------Red-------------*/
	{
		0, // 200,//20,			     //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX
	},
	/*------------Origin-------------*/
	{
		0, // 200,//20,			     //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX

	},
	/*------------Blue-------------*/
	{
		0, // 200,//20,			      //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		//��Ҫ���� ����������Ͷȷ�Χ�ڡ� �����塣
		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		//��Ҫ���� �����������(�Ҷ�)��Χ�ڡ� �����塣
		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		//��Ҫ���� ���������С��Χ�ڡ� �����塣
		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX
	},
	/*------------Greed-------------*/
	{
		0, // 200,//20,			     //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX
	},
	/*------------White-------------*/
	{
		0, // 200,//20,			     //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX
	},
	/*------------Yewllo-------------*/
	{

		0, // 200,//20,			      //Ŀ��ɫ�ȣ�H_MIN	 ��Щ�������ڵ����Դ���ͼ����������ҵ���ͬ��ɫ��ӦH��S��Lֵ��
		0, // 280,//160,    		//Ŀ��ɫ�ȣ�H_MAX  ��ɫ160  ��ɫ80

		//��Ҫ���� ����������Ͷȷ�Χ�ڡ� �����塣
		0, // 140,//50,    	 	//Ŀ����С���Ͷȣ�S_MIN
		0, // 250,//200,   	 	//Ŀ����󱥺Ͷȣ�S_MAX

		//��Ҫ���� �����������(�Ҷ�)��Χ�ڡ� �����塣
		0, // 0,//50,     			//Ŀ����С���ȣ�L_MIN
		0, // 100,//200,    		//Ŀ��������ȣ�L_MAX

		//��Ҫ���� ���������С��Χ�ڡ� �����塣
		0, //Ŀ����С��ȣ�WIDTH_MIN
		0, //Ŀ����С�߶ȣ�HEIGHT_MIN

		0, //Ŀ������ȣ�WIDTH_MAX
		0, //Ŀ�����߶ȣ�HEIGHT_MAX

	}};

SEARCH_AREA_t area[OBJECT_NUM + 1] = {IMG_X, IMG_X + IMG_W, IMG_Y, IMG_Y + IMG_H}; //�����������򣬿�ʼ��    ��ȫ�ּ�������ɫ,����������

//��ȡĳ�����ɫ
void ReadColor(uint16_t usX, uint16_t usY, COLOR_RGB_t *color_rgb)
{
	uint16_t rgb;
	x1 = usX;
	y1 = usY;
	rgb = LCD_READPOINT(usX, usY); //��ȡ��ɫ����

	//ת����ֵ��Ϊ[0,255]����ԭɫֵ
	color_rgb->Red = (uint8_t)((rgb & 0xF800) >> 8);
	color_rgb->Green = (uint8_t)((rgb & 0x07E0) >> 3);
	color_rgb->Blue = (uint8_t)((rgb & 0x001F) << 3);
}
/*************************************/
// RGBת��ΪHLS
// H:ɫ��
// L������
// S�����Ͷ�
void RGB2HSL(const COLOR_RGB_t *color_rgb, COLOR_HLS_t *color_hls)
{
	int r, g, b;
	int h, l, s;
	int max, min, dif;

	r = color_rgb->Red;
	g = color_rgb->Green;
	b = color_rgb->Blue;

	max = maxOf3Values(r, g, b);
	min = minOf3Values(r, g, b);
	dif = max - min;

	//����l������
	l = (max + min) * 240 / 255 / 2;

	//����h��ɫ��
	if (max == min) //�޶��� RGBһ��  �ڻҰ�
	{
		s = 0; //���Ͷ�0
		h = 0; //ɫ��0
	}
	else
	{
		/*����ɫ��h*/
		if (max == r) //���Rֵ���
		{
			if (min == b) // h����0��40
			{
				h = 40 * (g - b) / dif;
			}
			else if (min == g) // h����200��240
			{
				h = 40 * (g - b) / dif + 240;
			}
		}
		else if (max == g)
		{
			h = 40 * (b - r) / dif + 80;
		}
		else if (max == b)
		{
			h = 40 * (r - g) / dif + 160;
		}

		//���㱥�Ͷ�s
		if (l == 0)
		{
			s = 0;
		}
		else if (l <= 120)
		{
			s = dif * 240 / (max + min);
		}
		else
		{
			// s = dif * 240 / ( 480 - ( max + min ) );
			s = (dif)*240 / (511 - (max + min));
		}
	}
	color_hls->Hue = h;		   //ɫ��
	color_hls->Lightness = l;  //����
	color_hls->Saturation = s; //���Ͷ�
}

/************************************************/
//  ��ɫƥ��
// color_hls ��COLOR_HLS�ṹ�壬�洢HLS��ʽ��ɫ����
// condition ��TARGET_CONDITION�ṹ�壬���ϣ������ɫ������ֵ
//  1�����ص���ɫ��Ŀ�귶Χ�ڣ�0�����ص���ɫ����Ŀ�귶Χ�ڡ�
int ColorMatch(const COLOR_HLS_t *color_hls, const TARGET_CONDITION_t *condition)
{
	u16 i = 0;
	i = 0;
	i |= color_hls->Lightness;

	if (color_hls->Hue >= condition->H_MIN &&
		color_hls->Hue <= condition->H_MAX) //��ɫ�ڷ�Χ��
	{
		if (
			((color_hls->Lightness >= condition->L_MIN) &&
			 (color_hls->Lightness <= condition->L_MAX)) &&
			((color_hls->Saturation >= condition->S_MIN) &&
			 (color_hls->Saturation <= condition->S_MAX)))
		{
			return 1;
		}
	}

	return 0;
}

int Draw(u16 x0, u16 y0, u8 r)
{
	int a, b;
	int di;
	uint16_t FailCount = 0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
	a = 0;
	b = r;
	di = 3 - (r << 1); //�ж��¸���λ�õı�־
	while (a <= b)
	{

		a++;
		//ʹ��Bresenham�㷨��Բ
		if (di < 0)
			di += 4 * a + 6;
		else
		{
			di += 10 + 4 * (a - b);
			b--;
		}
	}

	ReadColor(x0 - a - 5, y0 - b - 6, &rgb); //���Ͻ�
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 + a + 5, y0 - b - 6, &rgb); //���Ͻ�
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 - a - 5, y0 + b + 6, &rgb); //���½�
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 + a + 5, y0 + b + 6, &rgb); //���½�
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;
	return FailCount;
}

/****************************************************/
//  Ѱ�Ҹ�ʴ����
//  x ����ʴ����x����
//  y ����ʴ����y����
//  condition ��TARGET_CONDITION�ṹ�壬���ϣ������ɫ������ֵ
//  area ��SEARCH_AREA�ṹ�壬���Ҹ�ʴ���ĵ�����
// 1���ҵ��˸�ʴ���ģ�x��yΪ��ʴ���ĵ����ꣻ0��û���ҵ���ʴ���ġ�
uint16_t i, j, k;
uint16_t FailCount = 0; //ʧ�ܼ���
uint16_t SpaceX, SpaceY;
int SearchCenter(uint16_t *x, uint16_t *y, const TARGET_CONDITION_t *condition, SEARCH_AREA_t *area)
{

	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;

	SpaceX = Minimum_step; //����С��ȳ���3 Ϊ �����ѯ�Ĳ�����һ����λ      ʶ�����С���Ϊ40 40/3=13.333 ���൱��16�����ص�Ϊһ����С������Ͳ���ʶ��Χ��
	SpaceY = Minimum_step; //����С�߶ȳ���3 Ϊ ��ֱ��ѯ�Ĳ�����һ����λ			ʶ�����С�߶�Ϊ40 ͬ���һ��

	/* һ��һ��13*13��ɫ����search ��ÿ�ζ�����search color block Y���м��е�
		13/2=6 ����һ�������һ���г���6���pixel
		ƥ��ɹ��Ļ��Ͳ��˳�����ѭ����k����13���
		ת����X���м���ǵ�Ϊ��̬�㣬Y��Ϊ����  a[][];
	*/
	/*���򲽽���λ+��ֱ������λ �����һ�����ε�ɫ��*/			   //��С��width ��hight ��unitΪ13  �Ǿ���һһ��13multipy by 13 �ľ���Ϊһ��search region
	for (i = area->Y_Start; i < area->Y_End - SpaceY; i += SpaceY) //��һ��ѭ����  i����ͼƬ���ص�y��start��   �յ���  photo hight  ɨ�跽ʽΪ��ɨ��searchһ�о�Y�ͼ�һ������һ�е�search   ����Ϊ13pixel
	{
		for (j = area->X_Start + SpaceX; j < area->X_End - SpaceX; j += SpaceX) //�ڶ���ѭ����  i����ͼƬ���ص�X��start��   �յ���  photo width  	����Ϊ13pixel
		{
			FailCount = 0; //��ʼ��ʧ�ܼ���
			for (k = 0; k < SpaceX + SpaceY; k++)
			{
				x2 = j;
				y2 = i;
				kk = k;
				if (k < SpaceX)								// K�Ƿ�
					ReadColor(j + k, i + SpaceY / 2, &rgb); //��z
				else
				{
					x3 = j + SpaceX / 2;
					y3 = i + k - SpaceX;
					ReadColor(j + SpaceX / 2, i + k - SpaceX, &rgb); //��ѯɫ���м�һ������ɫ  //frist into j=13 j+spaceX/2=19
				}
				RGB2HSL(&rgb, &hls);

				if (!ColorMatch(&hls, condition))
					FailCount++; //��ɫ��ƥ�� ʧ�ܼ���+1

				if (FailCount > fail_flag) //ʧ�ܼ�������6   13��ѭ������6��û��ʶ��õ��Ļ���break
					break;				   //ʧ�ܴ���̫�� �˳�
			}

			if (k == SpaceX + SpaceY) // k=13+13����Ҳ����˵�ܺ��������ʶ����Ļ�˵�����ɫ����ʶ��ɹ��˵�
			{
				/*��¼��ɫ������ĵ�Ϊ��ʴ����*/
				*x = j - SpaceX;
				*y = i + SpaceY / 2;
				return 1; //��¼����һ����ʴ���ĺ��˳�����
			}
		}
	}

	return 0;
}
/*
ʹ��Bresenham�㷨�������Բ�λ��Ǿ��λ�Բ,����ʶ������
*/
int Draw_Circle(u16 x0, u16 y0, u8 r, RESULT_t *result)
{
	int a, b;
	int di;
	u16 FailCount = 0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
	a = 0;
	b = r;
	di = 3 - (r << 1); //�ж��¸���λ�õı�־
	while (a <= b)
	{

		a++;
		//ʹ��Bresenham�㷨��Բ
		if (di < 0)
		{
			di += 4 * a + 6;
		}
		else
		{
			di += 10 + 4 * (a - b);
			b--;

			ReadColor(x0 + a, y0 - b, &rgb);
			RGB2HSL(&rgb, &hls);
			if (!ColorMatch(&hls, &condition[0]))
				FailCount++;

			ReadColor(x0 + b, y0 - a, &rgb);
			RGB2HSL(&rgb, &hls);
			if (!ColorMatch(&hls, &condition[0]))
				FailCount++;
		}
	}
	return FailCount;
}
/*
�����ĵ㺯��
*/
void draw_cross(u16 x, u16 y)
{

	LCD_DrawLine(x - 5, y, x + 5, y);
	LCD_DrawLine(x, y - 5, x, y + 5);
}

/***************************************************/
// �Ӹ�ʴ�������ⸯʴ���õ��µĸ�ʴ����
//  oldX ����ǰ�ĸ�ʴ����x����
//  oldX ����ǰ�ĸ�ʴ����y����
//  condition ��TARGET_CONDITION�ṹ�壬���ϣ������ɫ������ֵ
//  result ��RESULT�ṹ�壬��ż����
// 1�����ɹ���0�����ʧ�ܡ�
uint16_t dail_flag = 4; //ʶ���ݴ���  ��С��ɫ��Ϊ40 ��ô40/20=2  ��Ҳ��ʱ˵���ʶ�𳬹�����ʧ�ܣ��ͽ�����ǰʶ��
#define count_stepping 1
int y_count_stepping = 13;
int x_count_stepping = 13;
int turn_flag = 3, distance_Y = 0, distance_X = 0;
uint16_t dir_flag = 0, border_flag = 0, last_dir_flag = 1, debug_flag = 0;
uint16_t Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, dis_x, dis_y = 0;
int Corrode(uint16_t oldX, uint16_t oldY, const TARGET_CONDITION_t *condition, RESULT_t *result)
{

	uint16_t i = 0, j = 0, aa = 0;
	uint16_t FailCount = 0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;

	// for(j=oldY+count_stepping;j>IMG_Y;j+=count_stepping)
	//	for(i=oldX+count_stepping; i>IMG_X; i-=count_stepping)						//�Ӿ�x������ʴ    ͼƬ��ʾ���120
	//	{
	//		ReadColor(i, oldY, &rgb);												//����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
	//		RGB2HSL(&rgb, &hls);														//ת��					//�Ѷ�ȡ����rgbֵ����ת��
	//		if(!ColorMatch(&hls, condition))								//ת����������ƤHLS
	//			FailCount++;																	//��ƥ������Լ�1
	//
	//         if( FailCount> condition->WIDTH_MIN/dail_flag )	//ʶ���ݴ���  ��С��ɫ��Ϊ40 ��ô40/4=10  ��Ҳ��ʱ˵���ʶ�𳬹�10��ʧ�ܣ��ͽ�����ǰʶ��
	//			break;
	//	}
	//
	//	Xmin=i;																						//����������ӵ�x����ߵ�ֵ  �ѱ���ɫ�������ֵ��������
	//
	//	FailCount=0;
	//	for(i=oldX; i<IMG_X+IMG_W; i+=count_stepping)			//�Ӿ�x�����Ҹ�ʴ   IMG_X=120 IMG_W=240 120+240=360  ͼƬ��ʾ�յ�
	//	{
	//		ReadColor(i, oldY, &rgb);												//�Ӹ�ʴ���ĵ��������Ҷ�ȡ��ɫ��
	//		RGB2HSL(&rgb, &hls);														//�Ѷ�ȡ����rgbֵ����ת��
	//		if(!ColorMatch(&hls, condition))								//ת����������ƤHLS
	//			FailCount++;

	//        if( FailCount> condition->WIDTH_MAX/dail_flag ) //ʶ���ݴ���  ��С��ɫ��Ϊ40 ��ô40/4=10  ��Ҳ��ʱ˵���ʶ�𳬹�10��ʧ�ܣ��ͽ�����ǰʶ��
	//			break;
	//	}
	//	FailCount=0;																			//ʧ�ܴ�������
	//	Xmax=i;																						//��������������õ�
	//	dis_y=oldY;																				//��������������õ�
	Ymax = Ymin = oldY; //��������������õ�
	Xmax = Xmin = oldX;
	/**************************************�㷨�޸�******************************************************/

	dir_flag = 1; //��ǰ��ʴ���� 1����  2����  3����
	border_flag = 0;
	last_dir_flag = 1; //��һ�θ�ʴ���� 1����  2����  3����
	i = oldX;
	j = oldY;
	//����Ѱ�ұ߽�
	while (1)
	{
		FailCount = 0; //����Ч������
		if (last_dir_flag == 1)
		{
			if (dir_flag == 1) //����ʴ
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ�� y1
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;
				}
				//���ڹ۲츯ʴ·��   �����޸�
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i > 2 * x_count_stepping) //�ж��Ƿ����㸯ʴ����
				{
					//			 last_dir_flag=1;
					//			 if(i<=x_count_stepping*2)
					//			 {i=x_count_stepping*2;}
					i = i - x_count_stepping;
					border_flag = 0;
				}
				else
				{
					if (border_flag == 0)
						last_dir_flag = 1;
					FailCount = 0;
					dir_flag = 2;
					border_flag++;
					i = i + x_count_stepping;
					j = j + y_count_stepping;
				}
				if (Xmin > i)
					Xmin = i; //��¼���������
			}
			else if (dir_flag == 2) //���¸�ʴ
			{

				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ�� y1
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //�Ӹ�ʴ���ĵ����������ȡ��ɫ��
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && j < condition->HEIGHT_MAX + oldY)
				{
					border_flag = 0;
					j = j + y_count_stepping;
					//			 last_dir_flag=2;
				}
				else
				{
					if (border_flag == 0)
					{
						last_dir_flag = 2;
						FailCount = 0;
						dir_flag = 1;
						border_flag++;
						j = j - y_count_stepping;
						if (i <= x_count_stepping * 2)
						{
							i = x_count_stepping * 2;
						}
						else
							i = i - x_count_stepping;
						//			 i=i-x_count_stepping;
					}
					else
						break;
				}
				if (Ymax < j)
					Ymax = j; //�ҵ��·��߽�
			}
		}
		else if (last_dir_flag == 2)
		{
			if (dir_flag == 1)
			{

				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫ�� y1
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫ��
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫ��
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ�������ص�
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i > 2 * x_count_stepping) //�����ȡ�����ص����turn_flag��Ը�������и�ʴ
				{
					border_flag = 0;
					i = i - x_count_stepping;
					//			 i=i-x_count_stepping;
				}
				else
				{
					FailCount = 0;
					if (border_flag == 0)
						last_dir_flag = 1;
					border_flag++;
					if (last_dir_flag == 2)
					{
						i = i + x_count_stepping * 2;
						dir_flag = 3;
					}
					else
					{
						j = j + y_count_stepping;
						dir_flag = 2;
						i = i + x_count_stepping;
					}
				}
				if (Xmin > i)
					Xmin = i; //��¼���������
			}
			else if (dir_flag == 3 && last_dir_flag == 2)
			{

				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i < WINDOW_WIDTH) //�����Ч�����turn_flag��Ը�������и�ʴ
				{
					//			 last_dir_flag=1;
					i = i + x_count_stepping;
					border_flag = 0;
				}
				else
				{
					if (border_flag == 0)
					{
						last_dir_flag = 3;
						FailCount = 0;
						dir_flag = 2;
						border_flag++;
						j = j + y_count_stepping;
						if (i <= x_count_stepping * 2)
						{
							i = x_count_stepping * 2;
						}
						else
							i = i - x_count_stepping;
						//			 	i=i-x_count_stepping;
					}
					else
						break;
				}
				if (Xmin > i)
					Xmin = i; //
			}
		}
		else if (last_dir_flag == 3)
		{

			if (dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && j < condition->HEIGHT_MAX + oldY)
				{
					//			 last_dir_flag=2;
					j = j + y_count_stepping;
					border_flag = 0;
				}
				else
				{
					if (border_flag == 0)
					{
						last_dir_flag = 2;
						dir_flag = 3;
						j = j - y_count_stepping;
						i = i + x_count_stepping;
						border_flag++;
					}
					else
						break;
				}
				if (Xmin > i)
					Xmin = i; //��¼���������
			}
		}
		if (border_flag >= 3 || j > condition->HEIGHT_MAX + oldY)
		{
			break;
		}
	}
	//Ѱ���ұ߽�
	dir_flag = 3; //��ǰ��ʴ���� 1����  2����  3����
	border_flag = 0;
	last_dir_flag = 3; //��һ�θ�ʴ���� 1����  2����  3����
	i = oldX;
	j = oldY;
	while (1)
	{

		//����Ѱ��
		//	for(j=dis_y;j<IMG_Y+IMG_H;j++)
		//	{
		FailCount = 0;
		if (last_dir_flag == 3)
		{
			if (dir_flag == 3)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i, j, i + y_count_stepping, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i < WINDOW_WIDTH)
				{
					//			 last_dir_flag=1;
					i = i + x_count_stepping;
					border_flag = 0;
				}
				else
				{
					if (border_flag == 0)
						last_dir_flag = 3;
					FailCount = 0;
					dir_flag = 2;
					border_flag++;
					if (i <= x_count_stepping * 2)
					{
						i = x_count_stepping * 2;
					}
					else
						i = i - x_count_stepping;
					//			 i=i-x_count_stepping;
					j = j + y_count_stepping;
					//			 }else break;
				}
				if (Xmax < i)
					Xmax = i; //��¼���ұ�����
			}
			else if (dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);							   //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);																	 //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i, j, i + y_count_stepping, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && j < condition->HEIGHT_MAX + oldY)
				{
					border_flag = 0;
					j = j + y_count_stepping;
				}
				else
				{
					FailCount = 0;
					if (border_flag == 0)
					{
						last_dir_flag = 2;
						dir_flag = 3;
						border_flag++;
						j = j - y_count_stepping;
						i = i + x_count_stepping;
					}
					else
						break;
				}
				//		 if(last_dir_flag==2)dir_flag=3;
				if (Ymax < j)
					Ymax = j; //�ҵ��·��߽�
			}
		}
		else if (last_dir_flag == 2)
		{
			if (dir_flag == 3)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i, j, i + y_count_stepping, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i < WINDOW_WIDTH)
				{
					border_flag = 0;
					i = i + x_count_stepping;
				}
				else
				{
					FailCount = 0;
					if (border_flag == 0)
						last_dir_flag = 3;
					border_flag++;
					if (last_dir_flag == 2)
					{
						i = i - x_count_stepping * 2;
						dir_flag = 1;
					}
					else
					{
						j = j + y_count_stepping;
						dir_flag = 2;
					}
				}
				if (Xmax < i)
					Xmax = i; //�ҵ��·��߽�
			}
			else if (dir_flag == 1 && last_dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i, j, i + y_count_stepping, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i > 2 * x_count_stepping)
				{
					//			 last_dir_flag=1;
					i = i - x_count_stepping;
					//			i=i-x_count_stepping;
					border_flag = 0;
				}
				else
				{
					FailCount = 0;
					if (border_flag == 0)
					{
						last_dir_flag = 1;
						dir_flag = 2;
						border_flag++;
						j = j + y_count_stepping;
						i = i + x_count_stepping;
					}
					else
						break;
				}
				if (Xmax < i)
					Xmax = i; //
			}
		}
		else if (last_dir_flag == 1)
		{

			//����Ѱ��

			if (dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //����      //��ȡ�ϱ߽���ɫֵ
						RGB2HSL(&rgb, &hls);		//ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //����      //��ȡ�±߽���ɫֵ
						RGB2HSL(&rgb, &hls);											  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //����      //��ȡ�ұ߽���ɫ��
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //����      //��ȡ��߽���ɫֵ
						RGB2HSL(&rgb, &hls);												  //ת��					//�Ѷ�ȡ����rgbֵ����ת��
					}
					if (ColorMatch(&hls, condition)) //ת����������ƤHLS
						FailCount++;				 //��ȡ������Ч��
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i, j, i + y_count_stepping, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && j < condition->HEIGHT_MAX + oldY)
				{
					//			 last_dir_flag=2;
					j = j + y_count_stepping;
					border_flag = 0;
				}
				else
				{
					if (border_flag == 0)
					{
						last_dir_flag = 2;
						dir_flag = 1;
						j = j - y_count_stepping;
						i = i + x_count_stepping;
						border_flag++;
					}
					else
						break;
				}
				if (Ymax < j)
					Ymax = j; //��¼���±�����
			}
		}
		if (border_flag >= 3 || j > condition->HEIGHT_MAX + oldY)
		{
			break;
		}
	}

	/******************************************************************************************************/
	/*
		�����ʶ��ɫ��Y�ᣨ��ȣ�
		*/
	//  for(i=Xmin;i<Xmax;i+=count_stepping)
	//	{
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//		for(j=dis_y;j>IMG_Y;j-=count_stepping)                                  //���ϸ�ʴ
	//	 {
	//		 ReadColor(i, j, &rgb);
	//		 RGB2HSL(&rgb, &hls);

	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount> condition->HEIGHT_MIN/dail_flag) 										//ʶ���ݴ���  ��С��ɫ��Ϊ40 ��ô40/20=2  ��Ҳ��ʱ˵���ʶ�𳬹�����ʧ�ܣ��ͽ�����ǰʶ��
	//				break;
	//	 }
	//	   if(j<Ymin)	Ymin=j;    	 																								//����ʶ�𵽵�Y�����Сֵ
	//	   FailCount=0;
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//		for(aa=dis_y;aa<IMG_Y+IMG_H;aa+=count_stepping)                         //���¸�ʴ
	//	 {
	//		 ReadColor(i, aa, &rgb);
	//		 RGB2HSL(&rgb, &hls);

	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount> condition->HEIGHT_MAX/dail_flag)
	//				break;
	//	  }
	//	   if(	aa>Ymax)								Ymax=aa;                       						//����ʶ�𵽵�Y������ֵ��distance_Y�Ǵ���Ymin and Ymax�Ĳ�ֵ��Ҳ��ʱ����ĸ߶�
	//	   if((Ymax-Ymin)>distance_Y) 	{distance_Y=Ymax-Ymin; dis_x=i;} 					//����ʶ�𵽵�������򣬼�¼��X������꣬�����ʹ�õ�
	//	    FailCount=0;
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//	}
	//
	///*
	//	�����ʶ��ɫ��X�ᣨ��ȣ���������ķ���һ��
	//	*/
	//
	//	for (i=Ymin;i<Ymax; i+=count_stepping)
	//	{
	//		for(j=dis_x;j>IMG_X;j-=count_stepping)                                 //����ʴ ��ֵ��С
	//	{
	//		 ReadColor(j, i, &rgb);
	//		 RGB2HSL(&rgb, &hls);
	//
	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount> dail_flag )
	//				break;
	//	}
	//	   if(	j<Xmin)	Xmin=j;
	//	   FailCount=0;

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	//		for(aa=dis_x;aa<IMG_X+IMG_W;aa+=count_stepping)                      //���Ҹ�ʴ ��ֵ���
	//	{
	//		 ReadColor(aa, i, &rgb);
	//		 RGB2HSL(&rgb, &hls);

	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount>dail_flag )
	//				break;
	//	}
	//	   if(	aa>Xmax)								Xmax=aa;
	//	   if((Xmax-Xmin)>distance_X) 	{distance_X=Xmax-Xmin;}
	//		  FailCount=0;

	//		 }

	// FailCount=0;

	//����ƫ�����Ը�һ����ֵ���仯��֪����
	Xmax += 5;
	Xmin -= 5;
	Ymax += 5;
	Ymin -= 5;
	distance_X = Xmax - Xmin;
	distance_Y = Ymax - Ymin;
	//����ƫ�����Ը�һ����ֵ���仯��֪����

	/***************************************************/
	//����ʶ�𲿷֣�����Բ�κ;��ε�ͼ���ص㣬����������ʶ��
	//ʶ��ɹ�����	FailCount<2 ��˵������Ǿ���
	// ReadColor(Xmin+3, Ymin+3, &rgb);                 //���Ͻ�
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;
	//
	// ReadColor(Xmax-3, Ymin+3, &rgb);                 //���Ͻ�
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;
	//
	// ReadColor(Xmin+3, Ymax, &rgb);                 	//���½�
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;

	// ReadColor(Xmax, Ymax-3, &rgb);                 //�����Ͻ�
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;

	// FailCount+= Draw(Xmin+(distance_X / 2),Ymin+(distance_Y / 2),distance_Y/2);              		//����ʶ��ڶ���ʶ��Բ����ߵ�4����
	// if(FailCount<2)
	//	{
	//	FailCount+=	Draw_Circle(Xmin+(distance_X / 2),Ymin+(distance_Y / 2),distance_Y/2,result);	//����ʶ��ڶ���ʶ��Բ�εıߵ���ɫ�Ƿ��ǰ�ɫ���������˵���Ǿ���
	// }
	// if(FailCount<2)
	//	{
	// result->object=	0;					//����ʶ����1Ϊ����  0ΪԲ��
	// object_flag=0;
	//	}
	// else
	//	{
	//	result->object=	1;	     //����ʶ����1Ϊ����  0ΪԲ��
	//	object_flag=1;
	//	}
	/***************************************************/
	//����ʶ�𲿷�
	//  Digital_recognition(Xmin,Xmax,Ymin,Ymax,condition);
	//���浱ǰɫ�����������
	result->x = Xmin + (distance_X / 2);
	result->y = Ymin + (distance_Y / 2);
	result->w = distance_X;
	result->h = distance_Y;
	result->Xmin = Xmin;
	result->Ymin = Ymin;
	result->Xmax = Xmax;
	result->Ymax = Ymax;
	// LCD_DrawRectangle(Xmin,Ymin,Xmax,Ymax);
	if ((result->w < condition->WIDTH_MAX) && (result->h < condition->HEIGHT_MAX) && (result->w > condition->WIDTH_MIN) && (result->h > condition->HEIGHT_MIN)) //�����ʴ�������û�г�������޶�������û��С����С�޶����� ��Ч����
		//	if( (result->w > condition->WIDTH_MIN/4) && (result->w < condition->WIDTH_MAX/4) &&
		//			(result->h > condition->HEIGHT_MIN) && (result->h < condition->HEIGHT_MAX)  )
		//
		return 1; //�����ʴ�������û�г�������޶�������û��С����С�޶����� ��Ч����
	else
		return 0;
}
//��������ʶ����
#if (Distinguish_Method == 1)
int X_distan = 0, x = 0, y = 0, y_distan = 0, count1 = 0, count2 = 1, task_count = 0;
u8 y1_count = 0, L_x1 = 0, R_x1 = 0, L_x2 = 0, R_x2 = 0, number = 0, task_count1 = 0, task_count2 = 0; //��������ʶ��ʹ��
int Digital_recognition(int x_min, int x_max, int y_min, int y_max, const TARGET_CONDITION_t *condition, u16 wide, u16 height)
{
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
	int x_flag = 0, y1_flag = 0, y2_flag = 0;
	y1_count = 0, L_x1 = 0, R_x1 = 0, L_x2 = 0, R_x2 = 0, y = 0;

	X_distan = x_max - x_min;
	y_distan = y_max - y_min;
	x_flag = x_min + X_distan / 2 - 1;
	y1_flag = y_min + y_distan * 2 / 3 - 1;
	y2_flag = y_min + y_distan * 2 / 5 - 1;
	//�ж�y1�����ݽ������
	count1 = 0, count2 = 1;
	for (x = x_flag; x < x_flag + 3; x++)
	{
		for (y = y_min; y < y_max; y++)
		{
			ReadColor(x, y, &rgb); //����
			RGB2HSL(&rgb, &hls);   //ת��
			if (ColorMatch(&hls, condition) && (count2 != 0))
			{
				count1++;
				if (count1 == 3)
					count2 = 0;
			}
			if (!ColorMatch(&hls, condition) && (count1 != 0))
			{

				count2++;
				if (count2 == 3)
				{
					y1_count++;
					count1 = 0;
				}
			}
		}
	}
	//�ж�x1�����ݽ������
	count1 = 0, count2 = 1;

	for (y = y1_flag; y < y1_flag + 3; y++)
	{
		for (x = x_min; x < x_max; x++)
		{
			ReadColor(x, y, &rgb); //����
			RGB2HSL(&rgb, &hls);   //ת��
			if (ColorMatch(&hls, condition) && (count2 != 0))
			{
				count1++;

				if (count1 == 3)
				{
					//					L_x1++;
					count2 = 0;
				}
			}
			if (!ColorMatch(&hls, condition) && (count1 != 0))
			{

				count2++;
				if (count2 == 4 || x >= Xmax)
				{
					if (x < Xmin + X_distan / 2)
						L_x1++;
					else
						R_x1++;
					count1 = 0;
				}
			}
		}
	}
	//�ж�x2�����ݽ������
	count1 = 0, count2 = 1;
	for (y = y2_flag; y < y2_flag + 3; y++)
	{
		for (x = x_min; x < x_max; x++)
		{
			ReadColor(x, y, &rgb); //����
			RGB2HSL(&rgb, &hls);   //ת��
			if (ColorMatch(&hls, condition) && (count2 != 0))
			{
				count1++;

				if (count1 == 3)
				{
					count2 = 0;
					//					L_x2++;
				}
			}
			if (!ColorMatch(&hls, condition) && (count1 != 0))
			{

				count2++;
				if (count2 == 4 || x >= Xmax)
				{
					if (x < Xmin + X_distan / 2)
						L_x2++;
					else
						R_x2++;
					count1 = 0;
				}
			}
		}
	}
	if (L_x1 != 0)
	{
		L_x1 = 1;
	}
	if (R_x1 != 0)
	{
		R_x1 = 1;
	}
	if (L_x2 != 0)
	{
		L_x2 = 1;
	}
	if (R_x2 != 0)
	{
		R_x2 = 1;
	}
	//�ж����ִ�С
	if (task_count1 == 0 || task_count2 == 0)
	{
		if (number != 1)
		{
			if ((y1_count == 3) && (L_x1 == 1) && (R_x1 == 0) && (L_x2 == 1) && (R_x2 == 0))
			{
				number = 1;
				task_count1++;
			}
			else if ((y1_count == 3) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
			{
				number = 1;
				task_count1++;
			}
		}
		if (number != 2)
		{
			if ((y1_count == 6) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 1) && (R_x2 == 0))
			{
				number = 2;
				task_count2++;
			}

			else if ((y1_count == 7) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
			{
				number = 2;
				task_count2++;
			}
			else if ((y1_count == 9) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
			{
				number = 2;
				task_count2++;
			}
			else if ((y1_count == 9) && (L_x1 == 1) && (R_x1 == 0) && (L_x2 == 0) && (R_x2 == 1))
			{
				number = 2;
				task_count2++;
			}
			//		else if((y1_count==6)&&(L_x1==1)&&(R_x1==0)&&(L_x2==1)&&(R_x2==1))
			//	{
			//	number=4;
			//	}
			else if ((y1_count == 6) && (L_x1 == 1) && (R_x1 == 0) && (L_x2 == 0) && (R_x2 == 1))
			{
				number = 2;
				task_count2++;
			}
		}
	}
	if (task_count1 != 0 && task_count2 != 0)
	{
		if ((y1_count == 6) && (L_x1 == 1) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
		{
			number = 3;
		}
		if ((y1_count == 7) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
		{
			number = 3;
		}
		else if ((y1_count == 6) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 1) && (R_x2 == 1))
		{
			number = 4;
		}
		//		else if((y1_count/3==3)&&(L_x1==1)&&(R_x1==0)&&(L_x2==0)&&(R_x2==1))
		//	{
		//	number=5;
		//	}
		else if ((y1_count == 6) && (L_x1 == 1) && (R_x1 == 1) && (L_x2 == 1) && (R_x2 == 1))
		{
			number = 6;
		}
		else if ((y1_count == 9) && (L_x1 == 1) && (R_x1 == 1) && (L_x2 == 1) && (R_x2 == 1))
		{
			number = 6;
		}
		else if ((y1_count == 6) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
		{
			number = 7;
		}
		else if ((y1_count == 3) && (L_x1 == 0) && (R_x1 == 1) && (L_x2 == 0) && (R_x2 == 1))
		{
			number = 7;
		}
		else if ((y1_count == 9) && (L_x1 == 1) && (R_x1 == 0) && (L_x2 == 0) && (R_x2 == 1))
		{
			number = 8;
		}
	}

	return number;
}
#elif (Distinguish_Method == 2)
typedef struct
{
	int width;
	int height;
	float f_x;
	float f_y;
	int i_x;
	int i_y;
} src;
typedef struct
{

	float wide_rate;   //������ű���
	float height_rate; //�߶����ű���
} image_processing;
// float each_weight[4];
// char x_add=1, y_add=1;
// int sda;
// u8  Original_image[20][50];
// u8  current_image[20][50];
bool dtaa = 12, write_or_not = 1;

int Digital_recognition(int x_min, int x_max, int y_min, int y_max, const TARGET_CONDITION_t *condition, u16 wide, u16 height)
{
	u16 x, y;

	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
#if (memory_data == 1)	 //ѵ������
	//	for(y=y_min;y<20;y++)
	//	{
	//	  for(x=x_min;x<50;x++)
	//		{
	//		 ReadColor(x, y, &rgb);                 //����
	//     RGB2HSL(&rgb, &hls); //ת��
	//		  if(ColorMatch(&hls, condition))
	//			{
	//			  dtaa=0;
	//			}
	//			else dtaa=1;
	//			Original_image[y][x]=	dtaa;
	//		}
	//	}
	//  Original_image[19][48]=121;
	//		if(write_or_not==1)
	//	{

	//	}
	//		 for(i=0;i<20;i++)
	//		{
	//		SD_ReadDisk(current_image[i],i*50,2);
	//		}
	//	sda=current_image[19][48];
#elif (memory_data == 2) //��������

#endif

	return 0;
}
//��ͼ������Ϊ��Ա�ͼ���Сһ��
void Enlarge_or_reduce()
{
	src dst_frame, src_frame; // dst_frame  Ŀ��ͼ��   src_frameԭͼ��
	image_processing image;
	image.height_rate = (float)dst_frame.height / src_frame.height; //����Ŀ��ͼ����ԭͼ��߶����ű���
	image.wide_rate = (float)dst_frame.width / src_frame.height;	//����Ŀ��ͼ����ԭͼ�������ű���

	for (int i = 0; i < dst_frame.width; i++)
	{

		//
		src_frame.f_x = fabs((i + 0.5) / image.wide_rate - 0.5);   //�ҵ�Ŀ��ͼ����ԭͼ���е�x����  Ϊ�����Ͳ�һ��������
		src_frame.f_y = fabs((j + 0.5) / image.height_rate - 0.5); //�ҵ�Ŀ��ͼ����ԭͼ���е�y���� Ϊ�����Ͳ�һ��������
		src_frame.i_x = (i + 0.5) / image.wide_rate - 0.5;		   //�ҵ�Ŀ��ͼ����ԭͼ���x��������
		src_frame.i_y = (j + 0.5) / image.height_rate - 0.5;	   //�ҵ�Ŀ��ͼ����ԭͼ���Ӧ��y��������

		//////������������ӦȨ��ֵ
		each_weight[leftup] = Original_image[src_frame.i_y][src_frame.i_x] * (src_frame.i_x + x_add - src_frame.f_x) * (src_frame.i_y + y_add - src_frame.f_y);	  //����
		each_weight[leftdown] = Original_image[src_frame.i_y][src_frame.i_x + x_add] * (src_frame.f_x - src_frame.i_x) * (src_frame.i_y + y_add - src_frame.f_y); //
		each_weight[rightup] = Original_image[src_frame.i_y + y_add][src_frame.i_x] * (src_frame.i_x + x_add - src_frame.f_x) * (src_frame.f_y - src_frame.i_y);
		each_weight[rightdown] = Original_image[src_frame.i_y + y_add][src_frame.i_x + x_add] * (src_frame.f_x - src_frame.i_x) * (src_frame.f_y - src_frame.i_y);
		if ((each_weight[leftup] + each_weight[leftdown] + each_weight[rightup] + each_weight[rightdown]) > 0.6) //
			current_image[j][i] = 1;
		else
			current_image[j][i] = 0;
	}
}

#endif

/*
 *����0ʶ��ʧ�ܣ�1�ɹ�
 *�õ�ƥ��ɫ�����Ϣ
 */
// trace�����٣�ʶ����ɫ���� condition��ʶ����ɫ������һ���ṹ����������洢��HLS��threshold��result����������ʶ�𵽵������������С
int Trace(const TARGET_CONDITION_t *condition, RESULT_t *result_final, SEARCH_AREA_t *area, u8 control_num)
{

	static uint16_t x0, y0; // x0andy0 ����ַ��ʶ�����Ǳߣ��������港ʴ���ĵ����꣬Ҳ����ɫ������
	RESULT_t result;		// create a RESUL_T type struct

	if (!SearchCenter(&x0, &y0, condition, area)) //Ѱ�Ҹ�ʴ����      area��ʶ������Ҳ����ͼƬ��LCD�ϵ���ʾ����
	{

		area->X_Start = IMG_X; //ʶ��ʧ������¸�ֵ����
		area->X_End = IMG_X + IMG_W;
		area->Y_Start = IMG_Y;
		area->Y_End = IMG_Y + IMG_H;
		return 0;
	}
	//�ҵ���ʴ���� �õ��е�
	result.x = x0;
	result.y = y0;

	if (Corrode(result.x, result.y, condition, &result)) //���¸�ʴ���ҵ�ɫ��Ĵ�С
	{
		//����ɫ�������λ������
		result_final->x = result.x;
		result_final->y = result.y;
		result_final->w = result.w;
		result_final->h = result.h;
		result_final->Xmin = result.Xmin;
		result_final->Ymin = result.Ymin;
		result_final->Xmax = result.Xmax;
		result_final->Ymax = result.Ymax;
		result_final->object = result.object; //�洢������״������

		if (control_num + 1 < OBJECT_NUM) //ʶ��ɹ�һ��֮������Ƕ�����Ļ������浱ǰ����Ymin���Ǹ���Ϊ��һ�ε�������ʼ�㣬���Ҫȥ��һ����ô����ʶ�������������壬��֪������������������
		{
			area[control_num + 1].X_Start = IMG_X;
			area[control_num + 1].Y_Start = result.Ymin;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}
/****************************************/
//������ɫʶ��㺯��
void Quandrant_init(RESULT_t *result_final)
{

	Quadrant_control.Up_x = result_final->x + 2;
	Quadrant_control.Up_y = result_final->Ymin + 20;

	Quadrant_control.Dowm_x = result_final->x + 2;
	Quadrant_control.Dowm_y = result_final->Ymax - 20;

	Quadrant_control.Left_x = result_final->Xmin + 20;
	Quadrant_control.Left_y = result_final->y;
	Quadrant_control.Right_x = result_final->Xmax - 20;
	Quadrant_control.Right_y = result_final->y;
}
/****************************************/
//��ɫʶ����
/*
condition   				HSL��ֵ�ṹ��
result_final				ʶ���������������
Quadrant_control		��ɫʶ���ṹ��

*/
int consend_color(const TARGET_CONDITION_t *condition, RESULT_t *result_final, Quadrant *Quadrant_control)
{
	u8 i = 0, count_fail = 0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
	Quandrant_init(result_final); //������ɫʶ��㺯��
	for (i = 0; i < 4; i++)		  //ʶ��4����
	{

		switch (i)
		{
		case 0:
			ReadColor(Quadrant_control->Up_x, Quadrant_control->Up_y, &rgb);
			break;
		case 1:
			ReadColor(Quadrant_control->Dowm_x, Quadrant_control->Dowm_y, &rgb);
			break;
		case 2:
			ReadColor(Quadrant_control->Left_x, Quadrant_control->Left_y, &rgb);
			break;
		case 3:
			ReadColor(Quadrant_control->Right_x, Quadrant_control->Right_y, &rgb);
			break;
		}

		RGB2HSL(&rgb, &hls);
		if (ColorMatch(&hls, condition))
			count_fail++; //ʶ��ɹ��˼���
	}

	ReadColor(result_final->x, result_final->y, &rgb); //ʶ�����ĵ���ɫ
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, condition))
		count_fail++;

	if (count_fail > 2) //���ʶ������ɹ����ξ���ɹ���Ȼ���˳�
	{
		return 1;
	}

	return 0;
}
