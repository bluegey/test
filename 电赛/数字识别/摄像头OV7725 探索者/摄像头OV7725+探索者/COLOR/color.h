#ifndef __COLOR_H
#define __COLOR_H	

#include "sys.h"
//#define write_or_not 0
#define  memory_data  1
#define minOf3Values( v1, v2, v3 )			( (v1<v2) ? ( (v1<v3) ? (v1) : (v3) ) \
								: ( (v2<v3) ? (v2) : (v3) ) )

#define maxOf3Values( v1, v2, v3 )			( (v1>v2) ? ( (v1>v3) ? (v1) : (v3) ) \
								: ( (v2>v3) ? (v2) : (v3) ) )
                                
typedef struct							//判定为目标的条件
{
	uint16_t H_MIN;				//目标最小色度
	uint16_t H_MAX;				//目标最大色度
    
	uint8_t S_MIN;				//目标最小饱和度
	uint8_t S_MAX;				//目标最大饱和度
	
	uint8_t L_MIN;				//目标最小亮度
	uint8_t L_MAX;				//目标最大亮度
	
	uint16_t WIDTH_MIN;		//目标最小宽度
	uint16_t HEIGHT_MIN;	//目标最小高度
	
	uint16_t WIDTH_MAX;		//目标最大宽度
	uint16_t HEIGHT_MAX;	//目标最大高度
}TARGET_CONDITION_t;

typedef struct					//搜寻区域
{
	uint16_t X_Start;
	uint16_t X_End;
	uint16_t Y_Start;
	uint16_t Y_End;
}SEARCH_AREA_t;


typedef struct				//结果
{
	uint16_t x;			//目标x坐标
	uint16_t y;			//目标y坐标
	uint16_t w;			//目标宽度
	uint16_t h;			//目标高度
	uint16_t Xmin;
	uint16_t Ymin;
	uint16_t Xmax;
	uint16_t Ymax;
	uint16_t object;
}RESULT_t;

typedef struct						//HLS颜色
{
	uint8_t Hue;					//色度	,[0,240]				
	uint8_t Lightness;		//亮度	,[0,240]	     
	uint8_t Saturation;		//饱和度,[0,240]	     
}COLOR_HLS_t;

typedef struct 						//RGB
{
	uint8_t Red;					// [0,255]
	uint8_t Green;        // [0,255]
	uint8_t Blue;         // [0,255]
}COLOR_RGB_t;

typedef struct  {

	uint16_t	Up_x;
	uint16_t	Up_y;
	uint16_t	Dowm_x;
	uint16_t	Dowm_y;
	uint16_t	Left_x;
	uint16_t	Left_y;
	uint16_t	Right_x;
	uint16_t	Right_y;
}Quadrant;


//读取某点的颜色
void ReadColor( uint16_t usX, uint16_t usY, COLOR_RGB_t* color_rgb );

//RGB转换为HLS
void RGB2HSL( const COLOR_RGB_t* color_rgb, COLOR_HLS_t* color_hls );

 //  颜色匹配
int ColorMatch(const COLOR_HLS_t* color_hls, const TARGET_CONDITION_t* condition );

//  寻找腐蚀中心
int SearchCenter(uint16_t* x, uint16_t* y, const TARGET_CONDITION_t* condition,  SEARCH_AREA_t* area );

// 从腐蚀中心向外腐蚀，得到新的腐蚀中心
int Corrode(uint16_t oldX, uint16_t oldY, const TARGET_CONDITION_t* condition, RESULT_t* result );

// 得到匹配色块的信息
int Trace(const TARGET_CONDITION_t* condition, RESULT_t* result_final, SEARCH_AREA_t* area,u8 control_num);	

int Draw_Circle(u16 x0,u16 y0,u8 r, RESULT_t* result);

void Quandrant_init(RESULT_t* result_final);

int  consend_color(const TARGET_CONDITION_t* condition,RESULT_t* result_final,Quadrant* Quadrant_control );
int Digital_recognition(int x_min,int x_max,int y_min,int y_max,const TARGET_CONDITION_t* condition,u16 wide,u16 height);
int Draw(u16 x0,u16 y0,u8 r);
void draw_cross(u16 x,u16 y);
#endif
