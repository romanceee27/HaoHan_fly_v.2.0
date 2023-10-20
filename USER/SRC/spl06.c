#include "SPL06.h"

void SPL06_Write_Byte(u8 addr, u8 data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte(SPL06_Write);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(addr);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(data);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Stop();
}

u8 SPL06_Read_Byte(u8 addr)
{
    u8 SPL06_Data;

    MPU_IIC_Start();
    MPU_IIC_Send_Byte(SPL06_Write);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(addr);
    MPU_IIC_Wait_Ack();

    MPU_IIC_Start(); // start again
    MPU_IIC_Send_Byte(SPL06_Read);
    MPU_IIC_Wait_Ack();
    SPL06_Data = MPU_IIC_Read_Byte(0);
    MPU_IIC_Stop();
    return SPL06_Data;
}
u8 SPL06_Init(void)
{
    u8 SPL06_ID;
    SPL06_Write_Byte(RESET_Addr, 0x89); // Reset
    Delay_ms(100);
    SPL06_ID = SPL06_Read_Byte(ID_Addr);   // Read the SPL06's ID
    SPL06_Write_Byte(MEAS_CFG_Addr, 0x07); // Set Working mode and state of sensor
    SPL06_Write_Byte(PRS_CFG_Addr, 0x27);  // Set the PM-RATE and PM-PRC
    SPL06_Write_Byte(TMP_CFG_Addr, 0xA0);  // Set the TMPI-RATE and TMP-PRC
    SPL06_Write_Byte(CFG_REG_Addr, 0x04);  // Configuration of abort, measurement data shift and FIFO enable
    return SPL06_ID;
}

float Temperature_conversion(u32 Temp_Data, float k)
{
    float Temperature;
    int Temp;
    if (Temp_Data & 0x800000)
    {
        Temp = Temp_Data - Total_Number_24;
    }
    else
    {
        Temp = Temp_Data;
    }
    Temperature = Temp / k;
    return Temperature;
}

float Pressure_conversion(u32 Pressure_Data, float k)
{
    float Pressure;
    int Press;
    if (Pressure_Data & 0x800000)
    {
        Press = Pressure_Data - Total_Number_24;
    }
    else
    {
        Press = Pressure_Data;
    }
    Pressure = Press / k;
    return Pressure;
}

float Scale_factor(u8 Config_k)
{
    float k;
    switch (Config_k)
    {
    case 0:
        k = k_SPS1;
        break;
    case 1:
        k = k_SPS2;
        break;
    case 2:
        k = k_SPS4;
        break;
    case 3:
        k = k_SPS8;
        break;
    case 4:
        k = k_SPS16;
        break;
    case 5:
        k = k_SPS32;
        break;
    case 6:
        k = k_SPS64;
        break;
    case 7:
        k = k_SPS128;
        break;
    }
    return k;
}

void Parameter_Reading(int *Pressure_Para, int *Temperature_Para)
{
    u8 Temp_Config0, Temp_Config1, Temp_Config2;
    u8 Press_Config0, Press_Config1, Press_Config2, Press_Config3, Press_Config4;
    u8 Press_Config5, Press_Config6, Press_Config7, Press_Config8, Press_Config9;
    u8 Press_Config10, Press_Config11, Press_Config12, Press_Config13, Press_Config14;
    // Temperature
    Temp_Config0 = SPL06_Read_Byte(Temp_c0_Addr);
    Temp_Config1 = SPL06_Read_Byte(Temp_c1_Addr);
    Temp_Config2 = SPL06_Read_Byte(Temp_c2_Addr);
    Temperature_Para[0] = (Temp_Config0 << 4) + ((Temp_Config1 & 0xF0) >> 4);
    if (Temperature_Para[0] & 0x0800)
        Temperature_Para[0] = Temperature_Para[0] - Total_Number_12;
    Temperature_Para[1] = ((Temp_Config1 & 0x0F) << 8) + Temp_Config2;
    if (Temperature_Para[1] & 0x0800)
        Temperature_Para[1] = Temperature_Para[1] - Total_Number_12;
    // Pressure
    Press_Config0 = SPL06_Read_Byte(Press_c0_Addr);
    Press_Config1 = SPL06_Read_Byte(Press_c1_Addr);
    Press_Config2 = SPL06_Read_Byte(Press_c2_Addr);
    Press_Config3 = SPL06_Read_Byte(Press_c3_Addr);
    Press_Config4 = SPL06_Read_Byte(Press_c4_Addr);
    Press_Config5 = SPL06_Read_Byte(Press_c5_Addr);
    Press_Config6 = SPL06_Read_Byte(Press_c6_Addr);
    Press_Config7 = SPL06_Read_Byte(Press_c7_Addr);
    Press_Config8 = SPL06_Read_Byte(Press_c8_Addr);
    Press_Config9 = SPL06_Read_Byte(Press_c9_Addr);
    Press_Config10 = SPL06_Read_Byte(Press_c10_Addr);
    Press_Config11 = SPL06_Read_Byte(Press_c11_Addr);
    Press_Config12 = SPL06_Read_Byte(Press_c12_Addr);
    Press_Config13 = SPL06_Read_Byte(Press_c13_Addr);
    Press_Config14 = SPL06_Read_Byte(Press_c14_Addr);
    Pressure_Para[0] = (Press_Config0 << 12) + (Press_Config1 << 4) + ((Press_Config2 & 0xF0) >> 4); // c00
    if (Pressure_Para[0] & 0x80000)
        Pressure_Para[0] = Pressure_Para[0] - Total_Number_20;                                // c00
    Pressure_Para[1] = ((Press_Config2 & 0x0F) << 16) + (Press_Config3 << 8) + Press_Config4; // c10
    if (Pressure_Para[1] & 0x80000)
        Pressure_Para[1] = Pressure_Para[1] - Total_Number_20; // c10
    Pressure_Para[2] = (Press_Config5 << 8) + Press_Config6;   // c01
    if (Pressure_Para[2] & 0x8000)
        Pressure_Para[2] = Pressure_Para[2] - Total_Number_16; // c01
    Pressure_Para[3] = (Press_Config7 << 8) + Press_Config8;   // c11
    if (Pressure_Para[3] & 0x8000)
        Pressure_Para[3] = Pressure_Para[3] - Total_Number_16; // c11
    Pressure_Para[4] = (Press_Config9 << 8) + Press_Config10;  // c20
    if (Pressure_Para[4] & 0x8000)
        Pressure_Para[4] = Pressure_Para[4] - Total_Number_16; // c20
    Pressure_Para[5] = (Press_Config11 << 8) + Press_Config12; // c21
    if (Pressure_Para[5] & 0x8000)
        Pressure_Para[5] = Pressure_Para[5] - Total_Number_16; // c21
    Pressure_Para[6] = (Press_Config13 << 8) + Press_Config14; // c30
    if (Pressure_Para[6] & 0x8000)
        Pressure_Para[6] = Pressure_Para[6] - Total_Number_16; // c30
}
float Correcting_Pressure(int *Pressure_Para, float Pressure, float Temperature)
{
    float Corr_Pressure;
    Corr_Pressure = Pressure_Para[0] + Pressure * (Pressure_Para[1] + Pressure * (Pressure_Para[4] + Pressure * Pressure_Para[6])) + Temperature * Pressure_Para[2] + Temperature * Pressure * (Pressure_Para[3] + Pressure * Pressure_Para[5]);
    return Corr_Pressure;
}

float Correcting_Temperature(int *Temperature_Para, float Temperature)
{
    float Corr_Temperature;
    Corr_Temperature = Temperature_Para[0] * 0.5 + Temperature_Para[1] * Temperature;
    return Corr_Temperature;
}
