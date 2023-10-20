#ifndef __MPU6050_H
#define __MPU6050_H

#include "i2c.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
//#include "inv_mpu.h"
//#include "dmpKey.h"
//#include "dmpmap.h"
//#include "inv_mpu_dmp_motion_driver.h"
#include "filt.h"
//#include "stm32f4xx_rcc.h"

#include <math.h>

// MPU6050 AD0控制脚
#define MPU_AD0_CTRL PAout(15) // 控制AD0电平,从而控制MPU地址

// #define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
// #define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到

// 每一位对应功能
#define GYRO_OFFSET 0x01 // 第一位陀螺仪校准标志位
#define ACC_OFFSET 0x02  // 第二位加速度校准标志位
#define BAR_OFFSET 0x04  // 第三位气压计校准标志位
#define MAG_OFFSET 0x08  // 第四位磁力计校准标志位
#define FLY_ENABLE 0x10  // 第五位解锁上锁
#define WiFi_ONOFF 0x20  // 第六位WiFi开关
#define FLY_MODE 0x40    // 第七位模式选择(0:无头模式(默认) 1:有头模式)

// 对 SENSER_OFFSET_FLAG 的位的操作
#define SENSER_FLAG_SET(FLAG) SENSER_OFFSET_FLAG |= FLAG           // 标志位置1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG &= ~FLAG        // 标志位值0
#define GET_FLAG(FLAG) (SENSER_OFFSET_FLAG & FLAG) == FLAG ? 1 : 0 // 获取标志位状态

#define MPU_SELF_TESTX_REG 0X0D   // 自检寄存器X
#define MPU_SELF_TESTY_REG 0X0E   // 自检寄存器Y
#define MPU_SELF_TESTZ_REG 0X0F   // 自检寄存器Z
#define MPU_SELF_TESTA_REG 0X10   // 自检寄存器A
#define MPU_SAMPLE_RATE_REG 0X19  // 采样频率分频器
#define MPU_CFG_REG 0X1A          // 配置寄存器
#define MPU_GYRO_CFG_REG 0X1B     // 陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG 0X1C    // 加速度计配置寄存器
#define MPU_MOTION_DET_REG 0X1F   // 运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG 0X23      // FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG 0X24  // IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG 0X25 // IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG 0X26      // IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG 0X27 // IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG 0X28 // IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG 0X29      // IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG 0X2A // IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG 0X2B // IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG 0X2C      // IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG 0X2D // IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG 0X2E // IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG 0X2F      // IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG 0X30 // IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG 0X31 // IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG 0X32      // IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG 0X33   // IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG 0X34 // IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG 0X35   // IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG 0X36 // IIC主机状态寄存器
#define MPU_INTBP_CFG_REG 0X37  // 中断/旁路设置寄存器
#define MPU_INT_EN_REG 0X38     // 中断使能寄存器
#define MPU_INT_STA_REG 0X3A    // 中断状态寄存器

#define MPU_ACCEL_XOUTH_REG 0X3B // 加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG 0X3C // 加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG 0X3D // 加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG 0X3E // 加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG 0X3F // 加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG 0X40 // 加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG 0X41 // 温度值高八位寄存器
#define MPU_TEMP_OUTL_REG 0X42 // 温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG 0X43 // 陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG 0X44 // 陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG 0X45 // 陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG 0X46 // 陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG 0X47 // 陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG 0X48 // 陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG 0X63 // IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG 0X64 // IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG 0X65 // IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG 0X66 // IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG 0X67 // IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG 0X68  // 信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG 0X69 // 运动检测控制寄存器
#define MPU_USER_CTRL_REG 0X6A    // 用户控制寄存器
#define MPU_PWR_MGMT1_REG 0X6B    // 电源管理寄存器1
#define MPU_PWR_MGMT2_REG 0X6C    // 电源管理寄存器2
#define MPU_FIFO_CNTH_REG 0X72    // FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG 0X73    // FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG 0X74      // FIFO读写寄存器
#define MPU_DEVICE_ID_REG 0X75    // 器件ID寄存器

// 如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
// 如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR 0X68

// 因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)
// #define MPU_READ    				0XD1
// #define MPU_WRITE   				0XD0

extern uint8_t SENSER_OFFSET_FLAG;

// #define SENSER_FLAG_SET(FLAG) SENSER_OFFSET_FLAG |= FLAG            // 标志位置1
// #define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG &= ~FLAG         // 标志位值0
// #define GET_FLAG(FLAG) (SENSER_OFFSET_FLAG & FLAG) == FLAG ? 1 : 0; // 获取标志位状态

#define Kp_New 0.9f          // 互补滤波当前数据的权重
#define Kp_Old 0.1f          // 互补滤波历史数据的权重
#define Acc_Gain 0.0001220f  // 加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define Gyro_Gain 0.0609756f // 角速度变成度 (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65535.0)
#define Gyro_Gr 0.0010641f   // 角速度变成弧度(3.1415/180 * LSBg)
#define G 9.80665f           // m/s^2
#define RadtoDeg 57.324841f  // 弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad 0.0174533f // 角度到弧度 (角度 * 3.1415/180)

// 三轴整形，原始数据
//typedef struct 
//{
//    int16_t x;
//    int16_t y;
//    int16_t z;
//} int16_xyz;

//// 三轴浮点型
//typedef struct
//{
//    float x;
//    float y;
//    float z;
//} float_xyz;

//// 姿态解算后的欧拉角
//typedef struct
//{
//    float rol;
//    float yaw;
//    float pitch;
//} float_angle;

extern float_xyz g_rad, g_radold;            // 弧度制数据
extern float_xyz a_filt, a_filtold, g_filt;  // 滤波后的数值
extern float_angle anglt_real;               // 飞行姿态数据

u8 MPU_Init(void);                                  // 初始化MPU6050
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf); // IIC连续写
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);  // IIC连续读
u8 MPU_Write_Byte(u8 reg, u8 data);
u8 MPU_Read_Byte(uint8_t addr, uint8_t reg); // IIC写一个字节

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);

short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz);
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az);
void imu_update(float_xyz *Gyr_rad, float_xyz *Acc_filt, float_angle *Att_Angle);
void prepare_data(void);
static float invsqrt(float x);
void mpu_off(void);;
uint8_t mpu6050_offset(int16_xyz value, int16_xyz *offset, uint16_t sensivity);
void mpu6050_read(void);
void MPU6050_CalOff_Gyr(void);
void MPU6050_CalOff_Acc(void);
void MPU6050_CalOff(void);
#endif
