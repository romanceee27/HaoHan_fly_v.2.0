#include "i2c.h"

/**********************************************
函数名称：MPU_IIC_Delay
函数功能：MPU IIC延时函数，延时2ms
函数参数：无
函数返回值：无 
**********************************************/
void MPU_IIC_Delay(void)
{
    delay_us(4);
}

/**********************************************
函数名称：MPU_IIC_Init
函数功能：MPU IIC初始化
函数参数：无
函数返回值：无
**********************************************/
void MPU_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 先使能外设IO PORTB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;         // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        // IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                   // 根据设定参数初始化GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11); // PB10,PB11 输出高

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      // IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);                 // 根据设定参数初始化GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_7 | GPIO_Pin_8); // PB10,PB11 输出高

    GPIO_InitTypeDef GPIO_InitStructure;
}

/**********************************************
函数名称：MPU_IIC_Start
函数功能：MPU IIC发送起始信号
函数参数：无
函数返回值：无
**********************************************/
void MPU_IIC_Start(void)
{
    MPU_SDA_OUT(); // SDA线 输出
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SDA = 0; // START:当SCL线处于高电平时,SDA线突然从高变低,发送起始信号
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0; // 钳住I2C总线，准备发送或接收数据
}

/**********************************************
函数名称：MPU_IIC_Stop
函数功能：MPU IIC发送停止信号
函数参数：无
函数返回值：无
**********************************************/
void MPU_IIC_Stop(void)
{
    MPU_SDA_OUT(); // SDA线输出
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0; // STOP:当SCL线处于高电平时,SDA线突然从低变高,发送停止信号
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1; // 发送I2C总线结束信号
    MPU_IIC_Delay();
}

/**********************************************
函数名称：MPU_IIC_Wait_Ack
函数功能：MPU IIC等待信号到来
函数参数：无
函数返回值：1:接收应答信号成功  0:接收应答信号失败
**********************************************/
u8 MPU_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    MPU_SDA_IN(); // SDA设置为输入
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    while (MPU_READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL = 0; // 时钟输出0
    return 0;
}

/**********************************************
函数名称：MPU_IIC_Ack
函数功能：MPU IIC产生应答信号
函数参数：无
函数返回值：无
**********************************************/
void MPU_IIC_Ack(void)
{
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 0;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}

/**********************************************
函数名称：MPU_IIC_NAck
函数功能：MPU IIC不产生应答信号
函数参数：无
函数返回值：无
**********************************************/
void MPU_IIC_NAck(void)
{
    MPU_IIC_SCL = 0;
    MPU_SDA_OUT();
    MPU_IIC_SDA = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 1;
    MPU_IIC_Delay();
    MPU_IIC_SCL = 0;
}

/**********************************************
函数名称：MPU_IIC_Send_Byte
函数功能：MPU IIC发送一个字节
函数参数：txd：要发送的数据
函数返回值：无
主机发往从机
注意：IIC发送字节是一个一个位发送的，发送一个字节需要发送八次
**********************************************/
void MPU_IIC_Send_Byte(u8 txd)
{
    u8 t;
    MPU_SDA_OUT();
    MPU_IIC_SCL = 0; // 拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL = 1;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
    }
}

/**********************************************
函数名称：MPU_IIC_Read_Byte
函数功能：MPU IIC读取一个字节
函数参数：ack: 1,发送ACK   0,发送NACK
函数返回值：接收到的数据
注意：IIC读取字节是一个一个位读取的，读取一个字节需要读取八次
从机发往主机
**********************************************/
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    MPU_SDA_IN(); // SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        MPU_IIC_SCL = 0;
        MPU_IIC_Delay();
        MPU_IIC_SCL = 1;
        receive <<= 1;
        if (MPU_READ_SDA)
            receive++; // 如果读到了数据
        MPU_IIC_Delay();
    }
    if (!ack)
        MPU_IIC_NAck(); // 发送nACK
    else
        MPU_IIC_Ack(); // 发送ACK
    return receive;
}

// uint8_t IIC_write_byte_len(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
// {
//     uint8_t i = 0;
//     MPU_IIC_Start();
//     MPU_IIC_Send_Byte(dev);

//     if (MPU_IIC_Wait_Ack())
//     {
//         MPU_IIC_Stop();
//         return 1;
//     }

//     MPU_IIC_Send_Byte(reg);
//     MPU_IIC_Wait_Ack();

//     for (i = 0; i < len; i++)
//     {
//         MPU_IIC_Send_Byte(data[i]);
//         if (MPU_IIC_Wait_Ack())
//         {
//             MPU_IIC_Stop();
//             return 1;
//         }
//     }
//     MPU_IIC_Stop();
//     return 0;
// }

// 产生I2C起始信号(即START信号)
void IIC_Start(void)
{
    SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(4);
    IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL = 0;
}
// 产生I2C停止信号(即STOP信号)
void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL = 1;
    IIC_SDA = 1;
    delay_us(4);
}
// 等待应答信号
// 返回值：1，接收应答失败
//         0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN();
    IIC_SDA = 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL = 0;
    return 0;
}

// 产生ACK应答信号
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}
// 不产生ACK应答信号
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

// I2C发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL = 0;
    for (t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(2);
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}
// 读1个字节，ack=1时，发送ACK，ack=0时，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}
// #include "i2c.h"

// void i2c_init(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;
//     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      // 普通输出模式
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       // 上拉
//     GPIO_Init(GPIOB, &GPIO_InitStructure);             // 初始化

//     IIC_SCL = 1;
//     IIC_SDA = 1;
// }

// void i2c_start(void)
// {
//     SDA_OUT();
//     IIC_SCL = IIC_SDA = 1;
//     Delay_us(4);
//     IIC_SDA = 0; // IIC通信开启的标志
//     Delay_us(4);
//     IIC_SCL = 0; // 锁住IIC中线
// }

// void i2c_stop(void)
// {
//     SDA_OUT();
//     IIC_SCL1 = 0;
//     IIC_SDA = 0;
//     Delay_us(4);
//     IIC_SDA = 1; // IIC通信关闭
// }

// /**
//  * @brief 等待应答信号，返回值为1，没有收到应答；返回值为0，收到应答。
//  *
//  * @return u8
//  */
// u8 i2c_ack(void)
// {
//     u8 ucErrTime = 0;
//     SDA_IN(); // SDA 设置为输入
//     IIC_SDA = 1;
//     Delay_us(1);
//     IIC_SCL = 1;
//     Delay_us(1);
//     while (READ_SDA)
//     {
//         ucErrTime++;
//         if (ucErrTime > 250)
//         {
//             IIC_Stop();
//             return 1;
//         }
//     }

//     IIC_SCL = 0; // 时钟输出 0
//     return 0;
// }

// // 产生 ACK 应答
// void IIC_Ack(void)
// {
//     IIC_SCL = 0;
//     SDA_OUT();
//     IIC_SDA = 0;
//     delay_us(2);
//     IIC_SCL = 1;
//     delay_us(2);
//     IIC_SCL = 0;
// }

// // 不产生 ACK 应答
// void IIC_NAck(void)
// {
//     IIC_SCL = 0;
//     SDA_OUT();
//     IIC_SDA = 1;
//     delay_us(2);
//     IIC_SCL = 1;
//     delay_us(2);
//     IIC_SCL = 0;
// }

// // IIC 发送一个字节
// // 返回从机有无应答
// // 1，有应答
// // 0，无应答
// void IIC_Send_Byte(u8 txd)
// {
//     u8 t;
//     SDA_OUT();
//     IIC_SCL = 0; // 拉低时钟开始数据传输
//     for (t = 0; t < 8; t++)
//     {
//         IIC_SDA = (txd & 0x80) >> 7;
//         txd <<= 1;
//         delay_us(2); // 对 TEA5767 这三个延时都是必须的
//         IIC_SCL = 1;
//         delay_us(2);
//         IIC_SCL = 0;
//         delay_us(2);
//     }
// }
// // 读 1 个字节，ack=1 时，发送 ACK，ack=0，发送 nACK
// u8 IIC_Read_Byte(unsigned char ack)
// {
//     unsigned char i, receive = 0;
//     SDA_IN(); // SDA 设置为输入
//     for (i = 0; i < 8; i++)
//     {
//         IIC_SCL = 0;
//         delay_us(2);
//         IIC_SCL = 1;
//         receive <<= 1;
//         if (READ_SDA)
//             receive++;
//         delay_us(1);
//     }
//     if (!ack)
//         IIC_NAck(); // 发送 nACK
//     else
//         IIC_Ack(); // 发送 ACK
//     return receive;
// }
