#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
#include "stm32f4xx.h"
#include "delay.h"

// IO方向设置  ---PB11
#define MPU_SDA_IN()
{
    GPIOB->CRH &= 0XFFFF0FFF;
    GPIOB->CRH |= 8 << 12;
} // 上拉/下拉 输入模式
#define MPU_SDA_OUT()
{
    GPIOB->CRH &= 0XFFFF0FFF;
    GPIOB->CRH |= 3 << 12;
} // 推挽输出  输出模式

// IO操作函数
#define MPU_IIC_SCL PBout(10) // SCL
#define MPU_IIC_SDA PBout(11) // SDA
#define MPU_READ_SDA PBin(11) // 输入SDA

// IIC所有操作函数
void MPU_IIC_Delay(void);                // IIC延时2ms函数
void MPU_IIC_Init(void);                 // 初始化IIC的IO口
void MPU_IIC_Start(void);                // 发送IIC开始信号
void MPU_IIC_Stop(void);                 // 发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);          // IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack); // IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void);               // IIC等待ACK信号
void MPU_IIC_Ack(void);                  // IIC发送ACK信号
void MPU_IIC_NAck(void);                 // IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr, u8 addr);

#endif
