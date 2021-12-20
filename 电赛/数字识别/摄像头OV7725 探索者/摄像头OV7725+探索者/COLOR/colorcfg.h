#ifndef colorcfg_h
#define colorcfg_h

#include "color.h"


/*配置色块查询的范围  图像在LCD的坐标*/
#define IMG_X 40			      		//图片x坐标120
#define IMG_Y 40               //图片y坐标240 
#define IMG_W 200             	//图片宽度240
#define IMG_H 280            	 	//图片高度320

#define ALLOW_FAIL_PER       2   

#define COLOR_RANG           30    //设定颜色的偏移范围 越大越容易识别 太大容易误识别
#define COLOR_NUM            7     //设定追踪颜色的数目
#define OBJECT_NUM          1     //设定追踪物体数量
extern u8 global_page;									//当前颜色的
extern u8 color_list[COLOR_NUM][7+7];		//颜色字符
extern SEARCH_AREA_t area[OBJECT_NUM+1];							//定义搜索区域
extern RESULT_t result[OBJECT_NUM];			//定义搜索结果
extern TARGET_CONDITION_t condition[COLOR_NUM];//定义目标参数
extern Quadrant		Quadrant_control;

#define leftup      0//左上
#define leftdown    1//左下
#define rightup     2//右上
#define rightdown   3//右下
#define Distinguish_Method    1
#define LCD_READPOINT( usX, usY )  LCD_ReadPoint(usX,usY)//定义读点函数
void Enlarge_or_reduce(u16 wide,u16 height);
#endif

