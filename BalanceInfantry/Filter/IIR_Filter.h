/***/ /****************************************
       https://github.com/RagingWaves
 ----------------------------------------------
  * @file        IIR_Filter.c/h
  * @dependent   1.  IIR_fdacoefs.h 锟斤拷MATLAB锟斤拷锟缴的诧拷锟斤拷锟侥硷拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷前缀"IIR_"锟斤拷锟斤拷锟侥硷拷锟斤拷突锟斤拷
  *              2.  tmwtypes.h 锟斤拷锟斤拷锟藉看fdacoefs.h锟节碉拷说锟斤拷锟斤拷
  * @brief       IIR锟剿诧拷锟斤拷锟斤拷锟斤拷应使锟斤拷MATLAB锟斤拷fdatool锟斤拷锟斤拷锟斤拷锟缴的诧拷锟斤拷锟侥硷拷fdacoefs.h锟斤拷
  *              锟斤拷锟斤拷锟斤拷锟斤拷
  *              锟睫革拷锟剿诧拷锟斤拷锟斤拷锟斤拷锟斤拷
  *              使锟斤拷MATLAB锟斤拷fdatool锟斤拷锟斤拷锟铰碉拷.h锟侥硷拷直锟斤拷锟斤拷锟斤拷锟斤拷桑锟�
  *              锟斤拷锟藉方锟斤拷锟缴参匡拷锟斤拷锟斤拷锟斤拷锟斤拷拢锟�
  *              https://blog.csdn.net/zht9961020/article/details/6980981
  * @mark        1.  锟窖诧拷锟斤拷4锟斤拷2锟节★拷10锟斤拷5锟斤拷Butterworth锟斤拷锟斤拷锟斤拷确锟斤拷锟斤拷--22.3.29
  *              2.  锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷--22.4.2
  *              3.  锟斤拷锟斤拷头锟侥硷拷锟斤拷锟矫ｏ拷锟斤拷锟斤拷IIR_SECTIONS_BUF锟疥定锟斤拷
  * @version     V1.0.2
  * @auther      ZYuan
  * @url
 ----------------------------------------------
 ****************************************/
      /***/

#ifndef __IIR_FILTER_H
#define __IIR_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif
/**
 * @attention  锟斤拷锟斤拷锟叫★拷锟斤拷诮锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷小锟斤拷锟斤拷氡拷锟斤拷锟絋he buffer is too small锟斤拷锟斤拷
 *             锟斤拷锟斤拷锟斤拷为锟剿憋拷锟斤拷锟截革拷锟斤拷锟矫达拷锟侥硷拷时锟斤拷锟斤拷fdacoefs.h锟斤拷锟斤拷锟斤拷锟斤拷囟锟斤拷濉�
 */
#define IIR_SECTIONS_BUF 10

  typedef struct
  {
    double s_buf[IIR_SECTIONS_BUF + 1][2 + 1];
    double in;
    double out;
  } IIR_Filter_t;

  void IIR_Filter_Init(IIR_Filter_t *filter);
  double IIR_Filter_Calc(IIR_Filter_t *filter, double in);
#ifdef __cplusplus
}
#endif
#endif
