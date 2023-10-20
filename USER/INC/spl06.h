#ifndef __SPL06_H
#define __SPL06_H
#include "i2c.h"
#include "sys.h"
#include "delay.h"
#include "stdio.h"

#define SPL06_Write 0XEE
#define SPL06_Read 0xEF

#define k_SPS1 524288.0
#define k_SPS2 1572864.0
#define k_SPS4 3670016.0
#define k_SPS8 7864320.0
#define k_SPS16 253952.0
#define k_SPS32 516096.0
#define k_SPS64 1040384.0
#define k_SPS128 2088960.0

#define PSR_B2_Addr 0x00
#define PSR_B1_Addr 0x01
#define PSR_B0_Addr 0x02
#define TMP_B2_Addr 0x03
#define TMP_B1_Addr 0x04
#define TMP_B0_Addr 0x05
#define PRS_CFG_Addr 0x06  // 压强采集的采样速率与过采样速率配置
#define TMP_CFG_Addr 0x07  // 温度采集的采样速率与过采样速率配置
#define MEAS_CFG_Addr 0x08 // 用来读取当前寄存器的工作状态以及采集模式的配置
#define CFG_REG_Addr 0x09  // 配置中断以及压强数据与温度数据的位移
#define RESET_Addr 0x0C
#define ID_Addr 0x0D

#define Temp_c0_Addr 0x10
#define Temp_c1_Addr 0x11
#define Temp_c2_Addr 0x12

#define Press_c0_Addr 0x13
#define Press_c1_Addr 0x14
#define Press_c2_Addr 0x15
#define Press_c3_Addr 0x16
#define Press_c4_Addr 0x17
#define Press_c5_Addr 0x18
#define Press_c6_Addr 0x19
#define Press_c7_Addr 0x1A
#define Press_c8_Addr 0x1B
#define Press_c9_Addr 0x1C
#define Press_c10_Addr 0x1D
#define Press_c11_Addr 0x1E
#define Press_c12_Addr 0x1F
#define Press_c13_Addr 0x20
#define Press_c14_Addr 0x21

#define Total_Number_24 16777216.0
#define Total_Number_20 1048576.0
#define Total_Number_16 65536.0
#define Total_Number_12 4096.0

u8 SPL06_Init(void);
u8 SPL06_Read_Byte(u8 addr);
void SPL06_Write_Byte(u8 addr, u8 data);
void Parameter_Reading(int *Pressure_Para, int *Temperature_Para);
float Temperature_conversion(u32 Temp_Data, float k);
float Pressure_conversion(u32 Pressure_Data, float k);
float Scale_factor(u8 Config_k);
float Correcting_Pressure(int *Pressure_Para, float Pressure, float Temperature);
float Correcting_Temperature(int *Temperature_Para, float Temperature);

#endif
