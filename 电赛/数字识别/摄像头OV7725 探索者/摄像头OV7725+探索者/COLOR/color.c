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
		0,	// 200,//20,			      //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		10, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		//我要捕获 “在这个饱和度范围内” 的物体。
		0,	// 140,//50,    	 	//目标最小饱和度，S_MIN
		10, // 250,//200,   	 	//目标最大饱和度，S_MAX

		//我要捕获 “在这个亮度(灰度)范围内” 的物体。
		0,	// 0,//50,     			//目标最小亮度，L_MIN
		10, // 100,//200,    		//目标最大亮度，L_MAX

		//我要捕获 “在这个大小范围内” 的物体。
		35, //目标最小宽度，WIDTH_MIN
		60, //目标最小高度，HEIGHT_MIN

		160, //目标最大宽度，WIDTH_MAX
		260, //目标最大高度，HEIGHT_MAX
	},
	/*------------Red-------------*/
	{
		0, // 200,//20,			     //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX
	},
	/*------------Origin-------------*/
	{
		0, // 200,//20,			     //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX

	},
	/*------------Blue-------------*/
	{
		0, // 200,//20,			      //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		//我要捕获 “在这个饱和度范围内” 的物体。
		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		//我要捕获 “在这个亮度(灰度)范围内” 的物体。
		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		//我要捕获 “在这个大小范围内” 的物体。
		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX
	},
	/*------------Greed-------------*/
	{
		0, // 200,//20,			     //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX
	},
	/*------------White-------------*/
	{
		0, // 200,//20,			     //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX
	},
	/*------------Yewllo-------------*/
	{

		0, // 200,//20,			      //目标色度，H_MIN	 这些都可以在电脑自带画图软件上轻易找到不同颜色对应H、S、L值。
		0, // 280,//160,    		//目标色度，H_MAX  蓝色160  绿色80

		//我要捕获 “在这个饱和度范围内” 的物体。
		0, // 140,//50,    	 	//目标最小饱和度，S_MIN
		0, // 250,//200,   	 	//目标最大饱和度，S_MAX

		//我要捕获 “在这个亮度(灰度)范围内” 的物体。
		0, // 0,//50,     			//目标最小亮度，L_MIN
		0, // 100,//200,    		//目标最大亮度，L_MAX

		//我要捕获 “在这个大小范围内” 的物体。
		0, //目标最小宽度，WIDTH_MIN
		0, //目标最小高度，HEIGHT_MIN

		0, //目标最大宽度，WIDTH_MAX
		0, //目标最大高度，HEIGHT_MAX

	}};

SEARCH_AREA_t area[OBJECT_NUM + 1] = {IMG_X, IMG_X + IMG_W, IMG_Y, IMG_Y + IMG_H}; //定义搜索区域，开始在    中全局检索该颜色,并生成区域。

//读取某点的颜色
void ReadColor(uint16_t usX, uint16_t usY, COLOR_RGB_t *color_rgb)
{
	uint16_t rgb;
	x1 = usX;
	y1 = usY;
	rgb = LCD_READPOINT(usX, usY); //获取颜色数据

	//转换成值域为[0,255]的三原色值
	color_rgb->Red = (uint8_t)((rgb & 0xF800) >> 8);
	color_rgb->Green = (uint8_t)((rgb & 0x07E0) >> 3);
	color_rgb->Blue = (uint8_t)((rgb & 0x001F) << 3);
}
/*************************************/
// RGB转换为HLS
// H:色度
// L：亮度
// S：饱和度
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

	//计算l，亮度
	l = (max + min) * 240 / 255 / 2;

	//计算h，色度
	if (max == min) //无定义 RGB一样  黑灰白
	{
		s = 0; //饱和度0
		h = 0; //色度0
	}
	else
	{
		/*计算色度h*/
		if (max == r) //如果R值最大
		{
			if (min == b) // h介于0到40
			{
				h = 40 * (g - b) / dif;
			}
			else if (min == g) // h介于200到240
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

		//计算饱和度s
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
	color_hls->Hue = h;		   //色度
	color_hls->Lightness = l;  //亮度
	color_hls->Saturation = s; //饱和度
}

/************************************************/
//  颜色匹配
// color_hls ：COLOR_HLS结构体，存储HLS格式颜色数据
// condition ：TARGET_CONDITION结构体，存放希望的颜色数据阈值
//  1：像素点颜色在目标范围内；0：像素点颜色不在目标范围内。
int ColorMatch(const COLOR_HLS_t *color_hls, const TARGET_CONDITION_t *condition)
{
	u16 i = 0;
	i = 0;
	i |= color_hls->Lightness;

	if (color_hls->Hue >= condition->H_MIN &&
		color_hls->Hue <= condition->H_MAX) //颜色在范围内
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
	di = 3 - (r << 1); //判断下个点位置的标志
	while (a <= b)
	{

		a++;
		//使用Bresenham算法画圆
		if (di < 0)
			di += 4 * a + 6;
		else
		{
			di += 10 + 4 * (a - b);
			b--;
		}
	}

	ReadColor(x0 - a - 5, y0 - b - 6, &rgb); //左上角
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 + a + 5, y0 - b - 6, &rgb); //右上角
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 - a - 5, y0 + b + 6, &rgb); //左下角
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;

	ReadColor(x0 + a + 5, y0 + b + 6, &rgb); //右下角
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, &condition[0]))
		FailCount++;
	return FailCount;
}

/****************************************************/
//  寻找腐蚀中心
//  x ：腐蚀中心x坐标
//  y ：腐蚀中心y坐标
//  condition ：TARGET_CONDITION结构体，存放希望的颜色数据阈值
//  area ：SEARCH_AREA结构体，查找腐蚀中心的区域
// 1：找到了腐蚀中心，x、y为腐蚀中心的坐标；0：没有找到腐蚀中心。
uint16_t i, j, k;
uint16_t FailCount = 0; //失败计数
uint16_t SpaceX, SpaceY;
int SearchCenter(uint16_t *x, uint16_t *y, const TARGET_CONDITION_t *condition, SEARCH_AREA_t *area)
{

	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;

	SpaceX = Minimum_step; //以最小宽度除以3 为 横向查询的步进的一个单位      识别的最小宽度为40 40/3=13.333 就相当于16个像素的为一步进小于这个就不在识别范围内
	SpaceY = Minimum_step; //以最小高度除以3 为 垂直查询的步进的一个单位			识别的最小高度为40 同宽度一样

	/* 一个一个13*13的色块来search ，每次都是先search color block Y轴中间中点
		13/2=6 的那一行如果那一行有超过6点的pixel
		匹配成功的话就不退出当层循环。k等于13后就
		转换成X轴中间的那点为静态点，Y轴为变量  a[][];
	*/
	/*横向步进单位+垂直步进单位 组成了一个矩形的色块*/			   //最小的width 和hight 的unit为13  那就是一一个13multipy by 13 的矩形为一个search region
	for (i = area->Y_Start; i < area->Y_End - SpaceY; i += SpaceY) //第一个循环：  i等于图片像素的y轴start点   终点是  photo hight  扫描方式为行扫描search一行就Y就加一继续下一行的search   步进为13pixel
	{
		for (j = area->X_Start + SpaceX; j < area->X_End - SpaceX; j += SpaceX) //第二个循环：  i等于图片像素的X轴start点   终点是  photo width  	步进为13pixel
		{
			FailCount = 0; //初始化失败计数
			for (k = 0; k < SpaceX + SpaceY; k++)
			{
				x2 = j;
				y2 = i;
				kk = k;
				if (k < SpaceX)								// K是否
					ReadColor(j + k, i + SpaceY / 2, &rgb); //查z
				else
				{
					x3 = j + SpaceX / 2;
					y3 = i + k - SpaceX;
					ReadColor(j + SpaceX / 2, i + k - SpaceX, &rgb); //查询色块中间一竖的颜色  //frist into j=13 j+spaceX/2=19
				}
				RGB2HSL(&rgb, &hls);

				if (!ColorMatch(&hls, condition))
					FailCount++; //颜色不匹配 失败计数+1

				if (FailCount > fail_flag) //失败计数大于6   13次循环大于6次没有识别得到的话就break
					break;				   //失败次数太多 退出
			}

			if (k == SpaceX + SpaceY) // k=13+13，，也就是说能横和竖都能识别完的话说明这个色块是识别成功了的
			{
				/*记录该色块的中心点为腐蚀中心*/
				*x = j - SpaceX;
				*y = i + SpaceY / 2;
				return 1; //记录到第一个腐蚀中心后退出函数
			}
		}
	}

	return 0;
}
/*
使用Bresenham算法来辨别是圆形还是矩形画圆,特征识别区分
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
	di = 3 - (r << 1); //判断下个点位置的标志
	while (a <= b)
	{

		a++;
		//使用Bresenham算法画圆
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
画中心点函数
*/
void draw_cross(u16 x, u16 y)
{

	LCD_DrawLine(x - 5, y, x + 5, y);
	LCD_DrawLine(x, y - 5, x, y + 5);
}

/***************************************************/
// 从腐蚀中心向外腐蚀，得到新的腐蚀中心
//  oldX ：先前的腐蚀中心x坐标
//  oldX ：先前的腐蚀中心y坐标
//  condition ：TARGET_CONDITION结构体，存放希望的颜色数据阈值
//  result ：RESULT结构体，存放检测结果
// 1：检测成功；0：检测失败。
uint16_t dail_flag = 4; //识别容错率  最小的色块为40 那么40/20=2  ；也就时说如果识别超过两次失败，就结束当前识别
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
	//	for(i=oldX+count_stepping; i>IMG_X; i-=count_stepping)						//从旧x点向左腐蚀    图片显示起点120
	//	{
	//		ReadColor(i, oldY, &rgb);												//读点      //从腐蚀中心点坐标向左读取颜色，
	//		RGB2HSL(&rgb, &hls);														//转换					//把读取来的rgb值进行转换
	//		if(!ColorMatch(&hls, condition))								//转换完用来配皮HLS
	//			FailCount++;																	//不匹配计数自加1
	//
	//         if( FailCount> condition->WIDTH_MIN/dail_flag )	//识别容错率  最小的色块为40 那么40/4=10  ；也就时说如果识别超过10次失败，就结束当前识别
	//			break;
	//	}
	//
	//	Xmin=i;																						//获得最新蔓延的x最左边的值  把被别到色块的最左值保存起来
	//
	//	FailCount=0;
	//	for(i=oldX; i<IMG_X+IMG_W; i+=count_stepping)			//从旧x点向右腐蚀   IMG_X=120 IMG_W=240 120+240=360  图片显示终点
	//	{
	//		ReadColor(i, oldY, &rgb);												//从腐蚀中心点坐标向右读取颜色，
	//		RGB2HSL(&rgb, &hls);														//把读取来的rgb值进行转换
	//		if(!ColorMatch(&hls, condition))								//转换完用来配皮HLS
	//			FailCount++;

	//        if( FailCount> condition->WIDTH_MAX/dail_flag ) //识别容错率  最小的色块为40 那么40/4=10  ；也就时说如果识别超过10次失败，就结束当前识别
	//			break;
	//	}
	//	FailCount=0;																			//失败次数清零
	//	Xmax=i;																						//储存数据下面会用到
	//	dis_y=oldY;																				//储存数据下面会用到
	Ymax = Ymin = oldY; //储存数据下面会用到
	Xmax = Xmin = oldX;
	/**************************************算法修改******************************************************/

	dir_flag = 1; //当前腐蚀方向 1向左  2向下  3向右
	border_flag = 0;
	last_dir_flag = 1; //上一次腐蚀方向 1向左  2向下  3向右
	i = oldX;
	j = oldY;
	//向左寻找边界
	while (1)
	{
		FailCount = 0; //将有效点清零
		if (last_dir_flag == 1)
		{
			if (dir_flag == 1) //向左腐蚀
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色， y1
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;
				}
				//用于观察腐蚀路径   方便修改
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i > 2 * x_count_stepping) //判断是否满足腐蚀条件
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
					Xmin = i; //记录最左边坐标
			}
			else if (dir_flag == 2) //向下腐蚀
			{

				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色， y1
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //从腐蚀中心点坐标向左读取颜色，
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
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
					Ymax = j; //找到下方边界
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
						ReadColor(i - aa, j, &rgb); //读点      //读取上边界颜色， y1
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色，
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色，
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的像素点
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i > 2 * x_count_stepping) //如果读取到像素点大于turn_flag则对该区域进行腐蚀
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
					Xmin = i; //记录最左边坐标
			}
			else if (dir_flag == 3 && last_dir_flag == 2)
			{

				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i - aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
				}
#if DEBUG
				if (debug_flag == 1)
					LCD_DrawRectangle(i - x_count_stepping, j, i, j + y_count_stepping);
#endif
				if (FailCount > turn_flag && i < WINDOW_WIDTH) //如果有效点大于turn_flag则对该区域进行腐蚀
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
						ReadColor(i - aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i - aa + x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i - x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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
					Xmin = i; //记录最左边坐标
			}
		}
		if (border_flag >= 3 || j > condition->HEIGHT_MAX + oldY)
		{
			break;
		}
	}
	//寻找右边界
	dir_flag = 3; //当前腐蚀方向 1向左  2向下  3向右
	border_flag = 0;
	last_dir_flag = 3; //上一次腐蚀方向 1向左  2向下  3向右
	i = oldX;
	j = oldY;
	while (1)
	{

		//向左寻找
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
						ReadColor(i + aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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
					Xmax = i; //记录最右边坐标
			}
			else if (dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);							   //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);																	 //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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
					Ymax = j; //找到下方边界
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
						ReadColor(i + aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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
					Xmax = i; //找到下方边界
			}
			else if (dir_flag == 1 && last_dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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

			//向右寻找

			if (dir_flag == 2)
			{
				for (aa = 0; aa < (y_count_stepping + x_count_stepping) * 2; aa++)
				{

					if (aa < x_count_stepping)
					{
						ReadColor(i + aa, j, &rgb); //读点      //读取上边界颜色值
						RGB2HSL(&rgb, &hls);		//转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping && aa < x_count_stepping * 2)
					{
						ReadColor(i + aa - x_count_stepping, j + y_count_stepping, &rgb); //读点      //读取下边界颜色值
						RGB2HSL(&rgb, &hls);											  //转换					//把读取来的rgb值进行转换
					}
					else if (aa > x_count_stepping * 2 && aa < x_count_stepping * 2 + y_count_stepping)
					{
						ReadColor(i + x_count_stepping, j + aa - x_count_stepping * 2, &rgb); //读点      //读取右边界颜色，
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					else
					{
						ReadColor(i, j + aa - x_count_stepping * 2 - y_count_stepping, &rgb); //读点      //读取左边界颜色值
						RGB2HSL(&rgb, &hls);												  //转换					//把读取来的rgb值进行转换
					}
					if (ColorMatch(&hls, condition)) //转换完用来配皮HLS
						FailCount++;				 //读取到的有效点
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
					Ymax = j; //记录最下边坐标
			}
		}
		if (border_flag >= 3 || j > condition->HEIGHT_MAX + oldY)
		{
			break;
		}
	}

	/******************************************************************************************************/
	/*
		下面的识别色块Y轴（宽度）
		*/
	//  for(i=Xmin;i<Xmax;i+=count_stepping)
	//	{
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//		for(j=dis_y;j>IMG_Y;j-=count_stepping)                                  //向上腐蚀
	//	 {
	//		 ReadColor(i, j, &rgb);
	//		 RGB2HSL(&rgb, &hls);

	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount> condition->HEIGHT_MIN/dail_flag) 										//识别容错率  最小的色块为40 那么40/20=2  ；也就时说如果识别超过两次失败，就结束当前识别
	//				break;
	//	 }
	//	   if(j<Ymin)	Ymin=j;    	 																								//储存识别到的Y轴的最小值
	//	   FailCount=0;
	//
	///////////////////////////////////////////////////////////////////////////////////////////////////////
	//		for(aa=dis_y;aa<IMG_Y+IMG_H;aa+=count_stepping)                         //向下腐蚀
	//	 {
	//		 ReadColor(i, aa, &rgb);
	//		 RGB2HSL(&rgb, &hls);

	//		 if(!ColorMatch(&hls, condition))
	//				FailCount++;
	//
	//		 if( FailCount> condition->HEIGHT_MAX/dail_flag)
	//				break;
	//	  }
	//	   if(	aa>Ymax)								Ymax=aa;                       						//储存识别到的Y轴的最大值，distance_Y是储存Ymin and Ymax的差值，也就时物体的高度
	//	   if((Ymax-Ymin)>distance_Y) 	{distance_Y=Ymax-Ymin; dis_x=i;} 					//在能识别到的最高区域，记录下X轴的坐标，后面会使用到
	//	    FailCount=0;
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////
	//	}
	//
	///*
	//	下面的识别色块X轴（宽度），和上面的方法一样
	//	*/
	//
	//	for (i=Ymin;i<Ymax; i+=count_stepping)
	//	{
	//		for(j=dis_x;j>IMG_X;j-=count_stepping)                                 //向左腐蚀 数值变小
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
	//		for(aa=dis_x;aa<IMG_X+IMG_W;aa+=count_stepping)                      //向右腐蚀 数值变大
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

	//减少偏差，你可以改一下数值看变化就知道了
	Xmax += 5;
	Xmin -= 5;
	Ymax += 5;
	Ymin -= 5;
	distance_X = Xmax - Xmin;
	distance_Y = Ymax - Ymin;
	//减少偏差，你可以改一下数值看变化就知道了

	/***************************************************/
	//特征识别部分，根据圆形和矩形的图像特点，进行特征点识别
	//识别成功次数	FailCount<2 则说明这个是矩形
	// ReadColor(Xmin+3, Ymin+3, &rgb);                 //左上角
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;
	//
	// ReadColor(Xmax-3, Ymin+3, &rgb);                 //右上角
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;
	//
	// ReadColor(Xmin+3, Ymax, &rgb);                 	//左下角
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;

	// ReadColor(Xmax, Ymax-3, &rgb);                 //右下上角
	// RGB2HSL(&rgb, &hls);
	// if(ColorMatch(&hls, condition)) FailCount++ ;

	// FailCount+= Draw(Xmin+(distance_X / 2),Ymin+(distance_Y / 2),distance_Y/2);              		//特征识别第二部识别圆形外边的4个点
	// if(FailCount<2)
	//	{
	//	FailCount+=	Draw_Circle(Xmin+(distance_X / 2),Ymin+(distance_Y / 2),distance_Y/2,result);	//特征识别第二部识别圆形的边的颜色是否都是白色，如果不是说明是矩形
	// }
	// if(FailCount<2)
	//	{
	// result->object=	0;					//储存识别结果1为矩形  0为圆形
	// object_flag=0;
	//	}
	// else
	//	{
	//	result->object=	1;	     //储存识别结果1为矩形  0为圆形
	//	object_flag=1;
	//	}
	/***************************************************/
	//数字识别部分
	//  Digital_recognition(Xmin,Xmax,Ymin,Ymax,condition);
	//储存当前色块的坐标数据
	result->x = Xmin + (distance_X / 2);
	result->y = Ymin + (distance_Y / 2);
	result->w = distance_X;
	result->h = distance_Y;
	result->Xmin = Xmin;
	result->Ymin = Ymin;
	result->Xmax = Xmax;
	result->Ymax = Ymax;
	// LCD_DrawRectangle(Xmin,Ymin,Xmax,Ymax);
	if ((result->w < condition->WIDTH_MAX) && (result->h < condition->HEIGHT_MAX) && (result->w > condition->WIDTH_MIN) && (result->h > condition->HEIGHT_MIN)) //如果腐蚀后的区域没有超过最大限定区域且没有小于最小限定区域 有效！！
		//	if( (result->w > condition->WIDTH_MIN/4) && (result->w < condition->WIDTH_MAX/4) &&
		//			(result->h > condition->HEIGHT_MIN) && (result->h < condition->HEIGHT_MAX)  )
		//
		return 1; //如果腐蚀后的区域没有超过最大限定区域且没有小于最小限定区域 有效！！
	else
		return 0;
}
//数字特征识别函数
#if (Distinguish_Method == 1)
int X_distan = 0, x = 0, y = 0, y_distan = 0, count1 = 0, count2 = 1, task_count = 0;
u8 y1_count = 0, L_x1 = 0, R_x1 = 0, L_x2 = 0, R_x2 = 0, number = 0, task_count1 = 0, task_count2 = 0; //数字特征识别使用
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
	//判断y1与数据交点个数
	count1 = 0, count2 = 1;
	for (x = x_flag; x < x_flag + 3; x++)
	{
		for (y = y_min; y < y_max; y++)
		{
			ReadColor(x, y, &rgb); //读点
			RGB2HSL(&rgb, &hls);   //转换
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
	//判断x1与数据交点个数
	count1 = 0, count2 = 1;

	for (y = y1_flag; y < y1_flag + 3; y++)
	{
		for (x = x_min; x < x_max; x++)
		{
			ReadColor(x, y, &rgb); //读点
			RGB2HSL(&rgb, &hls);   //转换
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
	//判断x2与数据交点个数
	count1 = 0, count2 = 1;
	for (y = y2_flag; y < y2_flag + 3; y++)
	{
		for (x = x_min; x < x_max; x++)
		{
			ReadColor(x, y, &rgb); //读点
			RGB2HSL(&rgb, &hls);   //转换
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
	//判断数字大小
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

	float wide_rate;   //宽度缩放比例
	float height_rate; //高度缩放比例
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
#if (memory_data == 1)	 //训练部分
	//	for(y=y_min;y<20;y++)
	//	{
	//	  for(x=x_min;x<50;x++)
	//		{
	//		 ReadColor(x, y, &rgb);                 //读点
	//     RGB2HSL(&rgb, &hls); //转换
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
#elif (memory_data == 2) //比赛部分

#endif

	return 0;
}
//将图像缩放为与对比图像大小一致
void Enlarge_or_reduce()
{
	src dst_frame, src_frame; // dst_frame  目的图像   src_frame原图像
	image_processing image;
	image.height_rate = (float)dst_frame.height / src_frame.height; //计算目的图像与原图像高度缩放比例
	image.wide_rate = (float)dst_frame.width / src_frame.height;	//计算目的图像与原图像宽度缩放比例

	for (int i = 0; i < dst_frame.width; i++)
	{

		//
		src_frame.f_x = fabs((i + 0.5) / image.wide_rate - 0.5);   //找到目标图像在原图像中的x坐标  为浮点型不一定是整数
		src_frame.f_y = fabs((j + 0.5) / image.height_rate - 0.5); //找到目标图像在原图像中的y坐标 为浮点型不一定是整数
		src_frame.i_x = (i + 0.5) / image.wide_rate - 0.5;		   //找到目标图像在原图像的x整数坐标
		src_frame.i_y = (j + 0.5) / image.height_rate - 0.5;	   //找到目标图像在原图像对应的y整数坐标

		//////计算各点坐标对应权重值
		each_weight[leftup] = Original_image[src_frame.i_y][src_frame.i_x] * (src_frame.i_x + x_add - src_frame.f_x) * (src_frame.i_y + y_add - src_frame.f_y);	  //左上
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
 *返回0识别失败，1成功
 *得到匹配色块的信息
 */
// trace（跟踪）识别颜色函数 condition是识别颜色条件的一个结构体数组里面存储着HLS的threshold，result是用来储存识别到的物体坐标与大小
int Trace(const TARGET_CONDITION_t *condition, RESULT_t *result_final, SEARCH_AREA_t *area, u8 control_num)
{

	static uint16_t x0, y0; // x0andy0 传地址到识别函数那边，用来保存腐蚀中心的坐标，也就是色块中心
	RESULT_t result;		// create a RESUL_T type struct

	if (!SearchCenter(&x0, &y0, condition, area)) //寻找腐蚀中心      area是识别区域也就是图片在LCD上的显示区域
	{

		area->X_Start = IMG_X; //识别失败则从新赋值数据
		area->X_End = IMG_X + IMG_W;
		area->Y_Start = IMG_Y;
		area->Y_End = IMG_Y + IMG_H;
		return 0;
	}
	//找到腐蚀中心 得到中点
	result.x = x0;
	result.y = y0;

	if (Corrode(result.x, result.y, condition, &result)) //重新腐蚀，找到色块的大小
	{
		//储存色块的坐标位置数据
		result_final->x = result.x;
		result_final->y = result.y;
		result_final->w = result.w;
		result_final->h = result.h;
		result_final->Xmin = result.Xmin;
		result_final->Ymin = result.Ymin;
		result_final->Xmax = result.Xmax;
		result_final->Ymax = result.Ymax;
		result_final->object = result.object; //存储物体形状的数据

		if (control_num + 1 < OBJECT_NUM) //识别成功一次之后，如果是多物体的话，储存当前物体Ymin的那个点为下一次的搜索开始点，这个要去想一下怎么才能识别到两个或多个物体，就知道下下面程序的作用了
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
//定义颜色识别点函数
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
//颜色识别函数
/*
condition   				HSL数值结构体
result_final				识别物体的坐标数据
Quadrant_control		颜色识别点结构体

*/
int consend_color(const TARGET_CONDITION_t *condition, RESULT_t *result_final, Quadrant *Quadrant_control)
{
	u8 i = 0, count_fail = 0;
	COLOR_RGB_t rgb;
	COLOR_HLS_t hls;
	Quandrant_init(result_final); //定义颜色识别点函数
	for (i = 0; i < 4; i++)		  //识别4个点
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
			count_fail++; //识别成功了计数
	}

	ReadColor(result_final->x, result_final->y, &rgb); //识别中心点颜色
	RGB2HSL(&rgb, &hls);
	if (ColorMatch(&hls, condition))
		count_fail++;

	if (count_fail > 2) //如果识别次数成功两次就算成功，然后退出
	{
		return 1;
	}

	return 0;
}
