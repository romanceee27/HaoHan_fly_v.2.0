#include "filt.h"

#define N 20 // 滤波缓存数组大小
#define M_PI_F 3.1416f

/*******************************************************************************
 * 函  数 ：float FindPos(float*a,int low,int high)
 * 功  能 ：确定一个元素位序
 * 参  数 ：a  数组首地址
 *          low数组最小下标
 *          high数组最大下标
 * 返回值 ：返回元素的位序low
 * 备  注 : 无
 *******************************************************************************/
float FindPos(float *a, int low, int high)
{
    float val = a[low]; // 选定一个要确定值val确定位置
    while (low < high)
    {
        while (low < high && a[high] >= val)
            high--;       // 如果右边的数大于VAL下标往前移
        a[low] = a[high]; // 当右边的值小于VAL则复值给A[low]

        while (low < high && a[low] <= val)
            low++;        // 如果左边的数小于VAL下标往后移
        a[high] = a[low]; // 当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;
    return low;
}

/*******************************************************************************
 * 函  数 ：void QuiteSort(float* a,int low,int high)
 * 功  能 ：快速排序
 * 参  数 ：a  数组首地址
 *          low数组最小下标
 *          high数组最大下标
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
void QuiteSort(float *a, int low, int high)
{
    int pos;
    if (low < high)
    {
        pos = FindPos(a, low, high); // 排序一个位置
        QuiteSort(a, low, pos - 1);  // 递归调用
        QuiteSort(a, pos + 1, high);
    }
}

/*******************************************************************************
 * 函  数 ：float  SortAver_Filter(float value)
 * 功  能 ：去最值平均值滤波一组数据
 * 参  数 ：value 采样的数据
 *		   *filter 滤波以后的数据地址
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
void SortAver_Filter(float value, float *filter, uint8_t n)
{
    static float buf[N] = {0.0};
    static uint8_t cnt = 0, flag = 1;
    float temp = 0;
    uint8_t i = 0;
    buf[cnt++] = value;
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;
    QuiteSort(buf, 0, n - 1);
    for (i = 1; i < n - 1; i++)
    {
        temp += buf[i];
    }

    if (cnt >= n)
        cnt = 0;

    *filter = temp / (n - 2);
}

/*******************************************************************************
 * 函  数 ：float  SortAver_Filter1(float value)
 * 功  能 ：去最值平均值滤波一组数据
 * 参  数 ：value 采样的数据
 *		   *filter 滤波以后的数据地址
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
void SortAver_Filter1(float value, float *filter, uint8_t n)
{
    static float buf[N];
    static uint8_t cnt = 0, flag = 1;
    float temp = 0;
    uint8_t i;
    buf[cnt++] = value;
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;
    QuiteSort(buf, 0, n - 1);
    for (i = 1; i < n - 1; i++)
    {
        temp += buf[i];
    }
    if (cnt >= n)
        cnt = 0;

    *filter = temp / (n - 2);
}

/********************************************************************************
 * 函  数 ：void  SortAver_FilterXYZ(int16_xyz *acc,float_xyz *Acc_filt,uint8_t n)
 * 功  能 ：去最值平均值滤波三组数据
 * 参  数 ：*acc 要滤波数据地址
 *          *Acc_filt 滤波后数据地址
 * 返回值 ：无
 * 备  注 : 无
 ********************************************************************************/
void SortAver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t n)
{
    static float bufx[N], bufy[N], bufz[N];
    static uint8_t cnt = 0, flag = 1;
    float temp1 = 0, temp2 = 0, temp3 = 0;
    uint8_t i;
    bufx[cnt] = acc->x;
    bufy[cnt] = acc->y;
    bufz[cnt] = acc->z;
    cnt++; // 这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;

    QuiteSort(bufx, 0, n - 1);
    QuiteSort(bufy, 0, n - 1);
    QuiteSort(bufz, 0, n - 1);
    for (i = 1; i < n - 1; i++)
    {
        temp1 += bufx[i];
        temp2 += bufy[i];
        temp3 += bufz[i];
    }

    if (cnt >= n)
        cnt = 0;
    Acc_filt->x = temp1 / (n - 2);
    Acc_filt->y = temp2 / (n - 2);
    Acc_filt->z = temp3 / (n - 2);
}

/*********************************************************************************
* 函  数 ：void Aver_FilterXYZ6(int16_xyz *acc,int16_xyz *gry,float_xyz *Acc_filt,
                              float_xyz *Gry_filt,uint8_t n)
* 功  能 ：滑动窗口滤波六组数据
* 参  数 ：*acc ,*gry 要滤波数据地址
*          *Acc_filt,*Gry_filt 滤波后数据地址
* 返回值 ：无
* 备  注 : 无
*********************************************************************************/
void Aver_FilterXYZ6(int16_xyz *acc, int16_xyz *gry, float_xyz *Acc_filt, float_xyz *Gry_filt, uint8_t n)
{
    static float bufax[N], bufay[N], bufaz[N], bufgx[N], bufgy[N], bufgz[N];
    static uint8_t cnt = 0, flag = 1;
    float temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;
    uint8_t i;
    bufax[cnt] = acc->x;
    bufay[cnt] = acc->y;
    bufaz[cnt] = acc->z;
    bufgx[cnt] = gry->x;
    bufgy[cnt] = gry->y;
    bufgz[cnt] = gry->z;
    cnt++; // 这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;
    for (i = 0; i < n; i++)
    {
        temp1 += bufax[i];
        temp2 += bufay[i];
        temp3 += bufaz[i];

        temp4 += bufgx[i];
        temp5 += bufgy[i];
        temp6 += bufgz[i];
    }
    if (cnt >= n)
        cnt = 0;
    Acc_filt->x = temp1 / n;
    Acc_filt->y = temp2 / n;
    Acc_filt->z = temp3 / n;

    Gry_filt->x = temp4 / n;
    Gry_filt->y = temp5 / n;
    Gry_filt->z = temp6 / n;
}

/*******************************************************************************
 * 函  数 ：void Aver_FilterXYZ(int16_xyz *acc,float_xyz *Acc_filt,uint8_t n)
 * 功  能 ：滑动窗口滤波三组数据
 * 参  数 ：*acc  要滤波数据地址
 *          *Acc_filt 滤波后数据地址
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
void Aver_FilterXYZ(int16_xyz *acc, float_xyz *Acc_filt, uint8_t n)
{
    static int32_t bufax[N], bufay[N], bufaz[N];
    static uint8_t cnt = 0, flag = 1;
    int32_t temp1 = 0, temp2 = 0, temp3 = 0, i;
    bufax[cnt] = acc->x;
    bufay[cnt] = acc->y;
    bufaz[cnt] = acc->z;
    cnt++; // 这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;
    for (i = 0; i < n; i++)
    {
        temp1 += bufax[i];
        temp2 += bufay[i];
        temp3 += bufaz[i];
    }
    if (cnt >= n)
        cnt = 0;
    Acc_filt->x = temp1 / n;
    Acc_filt->y = temp2 / n;
    Acc_filt->z = temp3 / n;
}

/*******************************************************************************
 * 函  数 ：void Aver_Filter(float data,float *filt_data,uint8_t n
 * 功  能 ：滑动窗口滤波一组数据
 * 参  数 ：data  要滤波数据
 *          *filt_data 滤波后数据地址
 * 返回值 ：返回滤波后的数据
 * 备  注 : 无
 *******************************************************************************/
void Aver_Filter(float data, float *filt_data, uint8_t n)
{
    static float buf[N];
    static uint8_t cnt = 0, flag = 1;
    float temp = 0;
    uint8_t i;
    buf[cnt] = data;
    cnt++;
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;

    for (i = 0; i < n; i++)
    {
        temp += buf[i];
    }
    if (cnt >= n)
        cnt = 0;
    *filt_data = temp / n;
}

/*******************************************************************************
 * 函  数 ：void Aver_Filter1(float data,float *filt_data,uint8_t n
 * 功  能 ：滑动窗口滤波一组数据
 * 参  数 ：data  要滤波数据
 *          *filt_data 滤波后数据地址
 * 返回值 ：返回滤波后的数据
 * 备  注 : 无
 *******************************************************************************/
void Aver_Filter1(float data, float *filt_data, uint8_t n)
{
    static float buf[N];
    static uint8_t cnt = 0, flag = 1;
    float temp = 0;
    uint8_t i;
    buf[cnt++] = data;
    if (cnt < n && flag)
        return; // 数组填不满不计算
    else
        flag = 0;
    for (i = 0; i < n; i++)
    {
        temp += buf[i];
    }
    if (cnt >= n)
        cnt = 0;
    *filt_data = temp / n;
}



#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 这是一个低通滤波器函数，实现了一阶低通滤波器的功能。
 * 输入参数包括截止频率 hz、采样时间 time、输入信号 in 和输出信号的指针 out。
 */
void LPF_1(float hz, float time, float in, float *out)
{
    *out += (1 / (1 + 1 / (hz * 6.28f * time))) * (in - *out);
}

/**
 * @brief 这是限幅滤波器函数。
 * 输入参数包括采样时间 T、截止频率 hz、用于保存滤波器状态的结构体指针 data 和输入信号 in。
 */
void limit_filter(float T, float hz, _lf_t *data, float in)
{
    float abs_t;
    LPF_1(hz, T, in, &(data->lpf_1));
    abs_t = ABS(data->lpf_1);
    data->out = LIMIT(in, -abs_t, abs_t);
}
