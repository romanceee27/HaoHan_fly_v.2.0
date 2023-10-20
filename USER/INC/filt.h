#ifndef _FILT_H
#define _FILT_H

#include "stm32f4xx.h"
#include "sys.h"
#include "math.h"
//#include "mpu6050.h"

typedef struct
{
    float lpf_1;

    float out;
} _lf_t;


// 三轴整形，原始数据
typedef struct 
{
    int16_t x;
    int16_t y;
    int16_t z;
} int16_xyz;

// 三轴浮点型
typedef struct
{
    float x;
    float y;
    float z;
} float_xyz;

// 姿态解算后的欧拉角
typedef struct
{
    float rol;
    float yaw;
    float pitch;
} float_angle;

float FindPos(float *a, int low, int high);
void QuiteSort(float *a, int low, int high);
void SortAver_Filter(float value, float *filter, uint8_t n);
void SortAver_Filter1(float value, float *filter, uint8_t n);
void SortAver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t n);
void Aver_FilterXYZ6(int16_xyz *acc, int16_xyz *gry, float_xyz *Acc_filt, float_xyz *Gry_filt, uint8_t n);
void Aver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t n);
void Aver_Filter(float data, float *filt_data, uint8_t n);
void Aver_Filter1(float data, float *filt_data, uint8_t n);
void LPF_1(float hz, float time, float in, float *out);
void limit_filter(float T, float hz, _lf_t *data, float in);









//float Low_Filter(float value);
//void SortAver_Filter(float value, float *filter, uint8_t N);
//void SortAver_Filter1(float value, float *filter, uint8_t n);
//void SortAver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t N);
//void Aver_FilterXYZ6(int16_xyz *acc, int16_xyz *gry, float_xyz *Acc_filt, float_xyz *Gry_filt, uint8_t N);
//void Aver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t N);
//void Aver_Filter(float data, float *filt_data, uint8_t n);
//void Aver_Filter1(float data, float *filt_data, uint8_t n);
//void presssureFilter(float *in, float *out);

//void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
//float LPF2pApply_1(float sample);
#endif
