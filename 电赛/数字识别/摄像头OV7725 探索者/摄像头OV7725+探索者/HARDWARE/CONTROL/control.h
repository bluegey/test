#ifndef __gfp_control_H
#define __gfp_control_H	

#include "sys.h"



void OV7725_7670_Camera_Refresh_Phoenix(void);
void Key_Modify_Phoenix(void);

void OV7725_camera_refresh(void);
void OV7670_camera_refresh(void);
void Ov7725_7670_Choose_Init(void);	

//void draw_cross(u16 x,u16 y);
void color_assignment(void);
void color_Value(u16 x,u16 y,u8 object_num);

void PID_LR_figure_Phoenix(void);
void PID_UD_figure_Phoenix(void);
void Feature_Recognize_Start_Phoenix(void);

void Constant_Init(void);
void PWM_TEST(void);

void OV7725_camera_refresh_color_track(void);
void Draw_1(u16 x0,u16 y0,u8 r);

#endif
