/***/ /****************************************
	   https://github.com/RagingWaves
 ----------------------------------------------
  * @file        Smooth_Filter.c/h
  * @brief       ͨ�û���ƽ���˲���
  * @mark        1.  ���--22.4.8
  * @version     V1.0.0
  * @auther      ZYuan
  * @url
 ----------------------------------------------
 ****************************************/
	  /***/

#ifndef __SMOOTH_FILTER_H
#define __SMOOTH_FILTER_H
#ifdef __cplusplus
extern "C"
{
#endif
#define SMOOTH_FILTER_DATA_DP 8

	typedef struct
	{
#if SMOOTH_FILTER_DATA_DP > 8
		float raw_data[SMOOTH_FILTER_DATA_DP];
#else
	float raw_data[8];
#endif
		int dp;
		int pointer;
	} SmoothFilter_t;

	int SmoothFilter_Init(SmoothFilter_t *smooth_filter, int dp);
	float SmoothFilter_Calc(SmoothFilter_t *smooth_filter, float data);
#ifdef __cplusplus
}
#endif
#endif
