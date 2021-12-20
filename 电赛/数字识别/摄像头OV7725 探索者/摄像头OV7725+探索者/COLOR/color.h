#ifndef __COLOR_H
#define __COLOR_H	

#include "sys.h"
//#define write_or_not 0
#define  memory_data  1
#define minOf3Values( v1, v2, v3 )			( (v1<v2) ? ( (v1<v3) ? (v1) : (v3) ) \
								: ( (v2<v3) ? (v2) : (v3) ) )

#define maxOf3Values( v1, v2, v3 )			( (v1>v2) ? ( (v1>v3) ? (v1) : (v3) ) \
								: ( (v2>v3) ? (v2) : (v3) ) )
                                
typedef struct							//�ж�ΪĿ�������
{
	uint16_t H_MIN;				//Ŀ����Сɫ��
	uint16_t H_MAX;				//Ŀ�����ɫ��
    
	uint8_t S_MIN;				//Ŀ����С���Ͷ�
	uint8_t S_MAX;				//Ŀ����󱥺Ͷ�
	
	uint8_t L_MIN;				//Ŀ����С����
	uint8_t L_MAX;				//Ŀ���������
	
	uint16_t WIDTH_MIN;		//Ŀ����С���
	uint16_t HEIGHT_MIN;	//Ŀ����С�߶�
	
	uint16_t WIDTH_MAX;		//Ŀ�������
	uint16_t HEIGHT_MAX;	//Ŀ�����߶�
}TARGET_CONDITION_t;

typedef struct					//��Ѱ����
{
	uint16_t X_Start;
	uint16_t X_End;
	uint16_t Y_Start;
	uint16_t Y_End;
}SEARCH_AREA_t;


typedef struct				//���
{
	uint16_t x;			//Ŀ��x����
	uint16_t y;			//Ŀ��y����
	uint16_t w;			//Ŀ����
	uint16_t h;			//Ŀ��߶�
	uint16_t Xmin;
	uint16_t Ymin;
	uint16_t Xmax;
	uint16_t Ymax;
	uint16_t object;
}RESULT_t;

typedef struct						//HLS��ɫ
{
	uint8_t Hue;					//ɫ��	,[0,240]				
	uint8_t Lightness;		//����	,[0,240]	     
	uint8_t Saturation;		//���Ͷ�,[0,240]	     
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


//��ȡĳ�����ɫ
void ReadColor( uint16_t usX, uint16_t usY, COLOR_RGB_t* color_rgb );

//RGBת��ΪHLS
void RGB2HSL( const COLOR_RGB_t* color_rgb, COLOR_HLS_t* color_hls );

 //  ��ɫƥ��
int ColorMatch(const COLOR_HLS_t* color_hls, const TARGET_CONDITION_t* condition );

//  Ѱ�Ҹ�ʴ����
int SearchCenter(uint16_t* x, uint16_t* y, const TARGET_CONDITION_t* condition,  SEARCH_AREA_t* area );

// �Ӹ�ʴ�������ⸯʴ���õ��µĸ�ʴ����
int Corrode(uint16_t oldX, uint16_t oldY, const TARGET_CONDITION_t* condition, RESULT_t* result );

// �õ�ƥ��ɫ�����Ϣ
int Trace(const TARGET_CONDITION_t* condition, RESULT_t* result_final, SEARCH_AREA_t* area,u8 control_num);	

int Draw_Circle(u16 x0,u16 y0,u8 r, RESULT_t* result);

void Quandrant_init(RESULT_t* result_final);

int  consend_color(const TARGET_CONDITION_t* condition,RESULT_t* result_final,Quadrant* Quadrant_control );
int Digital_recognition(int x_min,int x_max,int y_min,int y_max,const TARGET_CONDITION_t* condition,u16 wide,u16 height);
int Draw(u16 x0,u16 y0,u8 r);
void draw_cross(u16 x,u16 y);
#endif
