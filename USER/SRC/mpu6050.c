#include "mpu6050.h"

static uint8_t mpu6050_buff[14];
int16_xyz g_offset_raw, a_offset_raw; // 零飘数据
int16_xyz g_raw, a_raw;               // 加速度和角速度原始值
float_angle anglt_real;               // 飞行姿态数据
float_xyz g_rad, g_radold;            // 弧度制数据
float_xyz a_filt, a_filtold, g_filt;  // 滤波后的数值
float accb[3], DCMgb[3][3];           // 方向矩阵
uint8_t SENSER_OFFSET_FLAG;
uint8_t acc_update = 0;

/**********************************************
函数名称：MPU_Init
函数功能：初始化MPU6050
函数参数：无
函数返回值：0,初始化成功  其他,初始化失败
**********************************************/
u8 MPU_Init(void)
{
    u8 res;

    //GPIO_InitTypeDef GPIO_InitStructure;

   // RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  // 使能AFIO时钟
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 先使能外设IO PORTA时钟
    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;        // 端口配置
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  // 推挽输出
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO口速度为50MHz
    // GPIO_Init(GPIOA, &GPIO_InitStructure);            // 根据设定参数初始化GPIOA

    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // 禁止JTAG,从而PA15可以做普通IO使用,否则PA15不能做普通IO!!!

    MPU_AD0_CTRL = 0; // 控制MPU6050的AD0脚为低电平,从机地址为:0X68

    MPU_IIC_Init();                          // 初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    Delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
    MPU_Set_Rate(50);                        // 设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效

    res = MPU_Read_Byte(MPU_SIGPATH_RST_REG, MPU_DEVICE_ID_REG);
    if (res == 0x68) // 器件ID正确,即res = addr = 0x68
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(50);                        // 设置采样率为50Hz
    }
    else
        return 1; // 地址设置错误,返回1
    return 0;     // 地址设置正确,返回0
}

/**********************************************
函数名称：MPU_Set_Gyro_Fsr
函数功能：设置MPU6050陀螺仪传感器满量程范围
函数参数：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}

/**********************************************
函数名称：MPU_Set_Accel_Fsr
函数功能：设置MPU6050加速度传感器满量程范围
函数参数：fsr:0,±2g;1,±4g;2,±8g;3,±16g
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

/**********************************************
函数名称：MPU_Set_LPF
函数功能：设置MPU6050的数字低通滤波器
函数参数：lpf:数字低通滤波频率(Hz)
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;

    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}

/**********************************************
函数名称：MPU_Set_Rate
函数功能：设置MPU6050的采样率(假定Fs=1KHz)
函数参数：rate:4~1000(Hz)  初始化中rate取50
函数返回值：0,设置成功  其他,设置失败
**********************************************/
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                     // 自动设置LPF为采样率的一半
}

/**********************************************
函数名称：MPU_Get_Temperature
函数功能：得到温度传感器值
函数参数：无
函数返回值：温度值(扩大了100倍)
**********************************************/
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;

    MPU_Read_Len(0X68, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((u16)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
}

/**********************************************
函数名称：MPU_Get_Gyroscope
函数功能：得到陀螺仪值(原始值)
函数参数：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
函数返回值：0,读取成功  其他,读取失败
**********************************************/
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6], res;

    res = MPU_Read_Len(0x68, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((u16)buf[0] << 8) | buf[1];
        *gy = ((u16)buf[2] << 8) | buf[3];
        *gz = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

/**********************************************
函数名称：MPU_Get_Accelerometer
函数功能：得到加速度值(原始值)
函数参数：ax,ay,az:加速度传感器x,y,z轴的原始读数(带符号)
函数返回值：0,读取成功  其他,读取失败
**********************************************/
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6], res;
    res = MPU_Read_Len(0x68, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((u16)buf[0] << 8) | buf[1];
        *ay = ((u16)buf[2] << 8) | buf[3];
        *az = ((u16)buf[4] << 8) | buf[5];
    }
    return res;
}

/**********************************************
函数名称：MPU_Write_Len
函数功能：IIC连续写(写器件地址、寄存器地址、数据)
函数参数：addr:器件地址      reg:寄存器地址
                 len:写入数据的长度  buf:数据区
函数返回值：0,写入成功  其他,写入失败
**********************************************/
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;

    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令(0为写,1为读)
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    for (i = 0; i < len; i++)
    {
        MPU_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU_IIC_Wait_Ack())    // 等待ACK
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}

/**********************************************
函数名称：MPU_Read_Len
函数功能：IIC连续读(写入器件地址后,读寄存器地址、数据)
函数参数：addr:器件地址        reg:要读的寄存器地址
                 len:要读取的数据长度  buf:读取到的数据存储区
函数返回值：0,读取成功  其他,读取失败
**********************************************/
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                 // 等待应答
    while (len)
    {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // 产生一个停止条件
    return 0;
}

/**********************************************
函数名称：MPU_Write_Byte
函数功能：IIC写一个字节
函数参数：data:写入的数据    reg:要写的寄存器地址
函数返回值：0,写入成功  其他,写入失败
**********************************************/
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((0x68 << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Send_Byte(data); // 发送数据
    if (MPU_IIC_Wait_Ack())  // 等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

/**********************************************
函数名称：MPU_Read_Byte
函数功能：IIC读一个字节
函数参数：addr:设备地址；reg:要读的寄存器地址
函数返回值：res:读取到的数据
**********************************************/
u8 MPU_Read_Byte(uint8_t addr, uint8_t reg)
{
    u8 res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(addr); // 发送器件地址+写命令
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(addr + 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();          // 等待应答
    res = MPU_IIC_Read_Byte(0);  // 读取数据,发送nACK
    MPU_IIC_Stop();              // 产生一个停止条件
    return res;
}

/******************************************************************************
 * 函  数：void MPU6050_CalOff(void)
 * 功  能：陀螺仪加速度校准
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
void MPU6050_CalOff(void)
{

    SENSER_FLAG_SET(ACC_OFFSET);  // 加速度校准
    SENSER_FLAG_SET(GYRO_OFFSET); // 陀螺仪校准
}

/******************************************************************************
 * 函  数：void MPU6050_CalOff_Acc(void)
 * 功  能：加速度计校准
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
void MPU6050_CalOff_Acc(void)
{
    SENSER_FLAG_SET(ACC_OFFSET); // 加速度校准
}

/******************************************************************************
 * 函  数：void MPU6050_CalOff_Gyr(void)
 * 功  能：陀螺仪校准
 * 参  数：无
 * 返回值：无
 * 备  注：无
 *******************************************************************************/
void MPU6050_CalOff_Gyr(void)
{
    SENSER_FLAG_SET(GYRO_OFFSET); // 陀螺仪校准
}

// 读取原始数据
void mpu6050_read(void)
{
    MPU_Read_Len(0x68, 0x3B, 14, mpu6050_buff);
}

// 零飘校准
uint8_t mpu6050_offset(int16_xyz value, int16_xyz *offset, uint16_t sensivity)
{
    static int32_t tempgx = 0, tempgy = 0, tempgz = 0;
    static uint16_t cnt_a;
    if (cnt_a == 0)
    {
        value.x = 0;
        value.y = 0;
        value.z = 0;

        tempgx = 0;
        tempgy = 0;
        tempgz = 0;
        cnt_a = 1;

        sensivity = 0;
        offset->x = 0;
        offset->y = 0;
        offset->z = 0;
    }

    tempgx += value.x;
    tempgy += value.y;
    tempgz += value.z - sensivity;
    if (cnt_a == 200)
    {
        offset->x = tempgx / cnt_a;
        offset->y = tempgy / cnt_a;
        offset->z = tempgy / cnt_a;
        cnt_a = 0;
        return 1;
    }

    cnt_a++;
    return 0;
}

// 去零偏
void mpu_off(void)
{
    // 加速度
    a_raw.x = ((((int16_t)mpu6050_buff[0]) << 8) | mpu6050_buff[1]) - a_offset_raw.x;
    a_raw.y = ((((int16_t)mpu6050_buff[2]) << 8) | mpu6050_buff[3]) - a_offset_raw.y;
    a_raw.z = ((((int16_t)mpu6050_buff[4]) << 8) | mpu6050_buff[5]) - a_offset_raw.z;

    // 角速度
    g_raw.x = ((((int16_t)mpu6050_buff[8]) << 8) | mpu6050_buff[9]) - g_offset_raw.x;
    g_raw.y = ((((int16_t)mpu6050_buff[10]) << 8) | mpu6050_buff[11]) - g_offset_raw.y;
    g_raw.z = ((((int16_t)mpu6050_buff[12]) << 8) | mpu6050_buff[13]) - g_offset_raw.z;

    if (GET_FLAG(GYRO_OFFSET))
    {
        if (mpu6050_offset(g_raw, &g_offset_raw, 0))
        {
            SENSER_FLAG_RESET(GYRO_OFFSET);

            // TODO：此处可能需要将零飘数据存到falsh

            SENSER_FLAG_SET(ACC_OFFSET); // 校准加速度
        }
    }

    if (GET_FLAG(GYRO_OFFSET)) // 加速度零飘校准
    {
        if (mpu6050_offset(a_raw, &g_offset_raw, 8196))
        {
            SENSER_FLAG_RESET(ACC_OFFSET);

            // TODO:同上
        }
    }
}

// 根号计算
static float invsqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 对陀螺仪的去零飘数据滤波赋予物理意义，为姿态解算准备
void prepare_data(void)
{
    static uint8_t IIR_mode = 1;
    mpu6050_read();
    mpu_off();
    SortAver_FilterXYZ(&a_raw, &a_filt, 12);

    // 加速度转为国际标准单位
    a_filt.x = (float)a_filt.x * Acc_Gain * G;
    a_filt.y = (float)a_filt.y * Acc_Gain * G;
    a_filt.z = (float)a_filt.z * Acc_Gain * G;

    // 陀螺仪转为弧度/秒
    g_rad.x = (float)g_raw.x * Gyro_Gr;
    g_rad.y = (float)g_raw.y * Gyro_Gr;
    g_rad.z = (float)g_raw.z * Gyro_Gr;

    if (IIR_mode)
    {
        a_filt.x = a_filt.x * Kp_New + a_filtold.x * Kp_Old;
        a_filt.y = a_filt.y * Kp_New + a_filtold.y * Kp_Old;
        a_filt.z = a_filt.z * Kp_New + a_filtold.z * Kp_Old;

        a_filtold.x = a_filt.x;
        a_filtold.y = a_filt.y;
        a_filtold.z = a_filt.z;
    }
    accb[0] = a_filt.x;
    accb[1] = a_filt.y;
    accb[2] = a_filt.z;

    if (accb[0] && accb[1] && accb[2])
    {
        acc_update = 1;
    }
}

#define Kp 1.50f     // proportional gain governs rate of convergence to accelerometer/magnetometer
                     // 比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.005f    // integral gain governs rate of convergence of gyroscope biases
                     // 积分增益控制陀螺偏差的收敛速度
#define halfT 0.005f // half the sample period 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0,ezInt = 0;

void imu_update(float_xyz *Gyr_rad, float_xyz *Acc_filt, float_angle *Att_Angle)
{
    uint8_t i;
    float matrix[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f}; // 初始化矩阵
    float ax = Acc_filt->x, ay = Acc_filt->y, az = Acc_filt->z;
    float gx = Gyr_rad->x, gy = Gyr_rad->y, gz = Gyr_rad->z;
    float vx, vy, vz;
    float ex, ey, ez;
    float norm;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    if (ax * ay * az == 0)
        return;

    // 加速度计测量的重力向量(机体坐标系)
    norm = invsqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

    // 陀螺仪积分估计重力向量(机体坐标系)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz);

    // 测量的重力向量与估算的重力向量差积求出向量间的误差
    ex = (ay * vz - az * vy); //+ (my*wz - mz*wy);
    ey = (az * vx - ax * vz); //+ (mz*wx - mx*wz);
    ez = (ax * vy - ay * vx); //+ (mx*wy - my*wx);

    // 用上面求出误差进行积分
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    // 将误差PI后补偿到陀螺仪
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt; // 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // 四元素的微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 单位化四元数
    norm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    // 矩阵R 将惯性坐标系(n)转换到机体坐标系(b)
    matrix[0] = q0q0 + q1q1 - q2q2 - q3q3; // 11(前列后行)
    matrix[1] = 2.f * (q1q2 + q0q3);       // 12
    matrix[2] = 2.f * (q1q3 - q0q2);       // 13
    matrix[3] = 2.f * (q1q2 - q0q3);       // 21
    matrix[4] = q0q0 - q1q1 + q2q2 - q3q3; // 22
    matrix[5] = 2.f * (q2q3 + q0q1);       // 23
    matrix[6] = 2.f * (q1q3 + q0q2);       // 31
    matrix[7] = 2.f * (q2q3 - q0q1);       // 32
    matrix[8] = q0q0 - q1q1 - q2q2 + q3q3; // 33

    // 四元数转换成欧拉角(Z->Y->X)
    Att_Angle->yaw += Gyr_rad->z * RadtoDeg * 0.01f;
    //	Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
    Att_Angle->rol = -asin(2.f * (q1q3 - q0q2)) * 57.3f;                                 // roll(负号要注意)
    Att_Angle->pitch = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * 57.3f; // pitch
    for (i = 0; i < 9; i++)
    {
        *(&(DCMgb[0][0]) + i) = matrix[i];
    }
    // TODO:可加失控保护
}
