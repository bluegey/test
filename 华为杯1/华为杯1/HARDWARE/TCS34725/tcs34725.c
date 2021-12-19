#include "tcs34725.h"
/******************************************************************************/

/******************************************************************************/
extern void delay_s(u32 i);

COLOR_RGBC rgb;
COLOR_HSL  hsl;

COLOR_RGBC rgb_1;
COLOR_HSL  hsl_1;

COLOR_RGBC rgb_2;
COLOR_HSL  hsl_2;

COLOR_RGBC rgb_3;
COLOR_HSL  hsl_3;

COLOR_RGBC rgb_4;
COLOR_HSL  hsl_4;
/******************************************************************************/
void TCS34725_I2C_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; //PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��
    GPIO_SetBits(GPIOB, GPIO_Pin_4 | GPIO_Pin_5); //�ߵ�ƽ
}
void TCS34725_I2C_Init_1()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��

    GPIO_SetBits(GPIOB, GPIO_Pin_3);  //�ߵ�ƽ
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
}
void TCS34725_I2C_Init_2()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; //PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��
    GPIO_SetBits(GPIOC, GPIO_Pin_11 | GPIO_Pin_12); //�ߵ�ƽ
}
void TCS34725_I2C_Init_3()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��

    GPIO_SetBits(GPIOA, GPIO_Pin_15);  //�ߵ�ƽ
    GPIO_SetBits(GPIOC, GPIO_Pin_10);
}
void TCS34725_I2C_Init_4()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //PB10/PB10=���I2C
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ͨ���������
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
    GPIO_Init(GPIOC, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��
    GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9); //�ߵ�ƽ
}
/*********************************************/
void TCS34725_I2C_Start()
{
    TCS_SDA_OUT();
    TCS_SDA_H;
    TCS_SCL_H;
    delay_s(40);//delay_us(4);
    TCS_SDA_L;
    delay_s(40);//delay_us(4);
    TCS_SCL_L;
}
void TCS34725_I2C_Start_1()
{
    TCS_SDA_OUT_1();
    TCS_SDA_H_1;
    TCS_SCL_H_1;
    delay_s(40);//delay_us(4);
    TCS_SDA_L_1;
    delay_s(40);//delay_us(4);
    TCS_SCL_L_1;
}
void TCS34725_I2C_Start_2()
{
    TCS_SDA_OUT_2();
    TCS_SDA_H_2;
    TCS_SCL_H_2;
    delay_s(40);//delay_us(4);
    TCS_SDA_L_2;
    delay_s(40);//delay_us(4);
    TCS_SCL_L_2;
}
void TCS34725_I2C_Start_3()
{
    TCS_SDA_OUT_3();
    TCS_SDA_H_3;
    TCS_SCL_H_3;
    delay_s(40);//delay_us(4);
    TCS_SDA_L_3;
    delay_s(40);//delay_us(4);
    TCS_SCL_L_3;
}
void TCS34725_I2C_Start_4()
{
    TCS_SDA_OUT_4();
    TCS_SDA_H_4;
    TCS_SCL_H_4;
    delay_s(40);//delay_us(4);
    TCS_SDA_L_4;
    delay_s(40);//delay_us(4);
    TCS_SCL_L_4;
}
/*********************************************/
void TCS34725_I2C_Stop()
{
    TCS_SDA_OUT();
    TCS_SCL_L;
    TCS_SDA_L;
    delay_s(40);//delay_us(4);
    TCS_SCL_H;
    TCS_SDA_H;
    delay_s(40);//delay_us(4);
}
void TCS34725_I2C_Stop_1()
{
    TCS_SDA_OUT_1();
    TCS_SCL_L_1;
    TCS_SDA_L_1;
    delay_s(40);//delay_us(4);
    TCS_SCL_H_1;
    TCS_SDA_H_1;
    delay_s(40);//delay_us(4);
}
void TCS34725_I2C_Stop_2()
{
    TCS_SDA_OUT_2();
    TCS_SCL_L_2;
    TCS_SDA_L_2;
    delay_s(40);//delay_us(4);
    TCS_SCL_H_2;
    TCS_SDA_H_2;
    delay_s(40);//delay_us(4);
}
void TCS34725_I2C_Stop_3()
{
    TCS_SDA_OUT_3();
    TCS_SCL_L_3;
    TCS_SDA_L_3;
    delay_s(40);//delay_us(4);
    TCS_SCL_H_3;
    TCS_SDA_H_3;
    delay_s(40);//delay_us(4);
}
void TCS34725_I2C_Stop_4()
{
    TCS_SDA_OUT_4();
    TCS_SCL_L_4;
    TCS_SDA_L_4;
    delay_s(40);//delay_us(4);
    TCS_SCL_H_4;
    TCS_SDA_H_4;
    delay_s(40);//delay_us(4);
}
/*********************************************/
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 TCS34725_I2C_Wait_ACK()
{
    u32 t = 0;

    TCS_SDA_IN();//SDA����Ϊ����
    TCS_SDA_H;
    delay_s(10);//delay_us(1);
    TCS_SCL_H;
    delay_s(10);//delay_us(1);

    while(TCS_SDA_READ)
    {
        t++;

        if(t > 250)
        {
            TCS34725_I2C_Stop();
            return 1;
        }
    }

    TCS_SCL_L;
    return 0;
}
u8 TCS34725_I2C_Wait_ACK_1()
{
    u32 t = 0;

    TCS_SDA_IN_1();//SDA����Ϊ����
    TCS_SDA_H_1;
    delay_s(10);//delay_us(1);
    TCS_SCL_H_1;
    delay_s(10);//delay_us(1);

    while(TCS_SDA_READ_1)
    {
        t++;

        if(t > 250)
        {
            TCS34725_I2C_Stop_1();
            return 1;
        }
    }

    TCS_SCL_L_1;
    return 0;
}
u8 TCS34725_I2C_Wait_ACK_2()
{
    u32 t = 0;

    TCS_SDA_IN_2();//SDA����Ϊ����
    TCS_SDA_H_2;
    delay_s(10);//delay_us(1);
    TCS_SCL_H_2;
    delay_s(10);//delay_us(1);

    while(TCS_SDA_READ_2)
    {
        t++;

        if(t > 250)
        {
            TCS34725_I2C_Stop_2();
            return 1;
        }
    }

    TCS_SCL_L_2;
    return 0;
}
u8 TCS34725_I2C_Wait_ACK_3()
{
    u32 t = 0;

    TCS_SDA_IN_3();//SDA����Ϊ����
    TCS_SDA_H_3;
    delay_s(10);//delay_us(1);
    TCS_SCL_H_3;
    delay_s(10);//delay_us(1);

    while(TCS_SDA_READ_3)
    {
        t++;

        if(t > 250)
        {
            TCS34725_I2C_Stop_3();
            return 1;
        }
    }

    TCS_SCL_L_3;
    return 0;
}
u8 TCS34725_I2C_Wait_ACK_4()
{
    u32 t = 0;

    TCS_SDA_IN_4();//SDA����Ϊ����
    TCS_SDA_H_4;
    delay_s(10);//delay_us(1);
    TCS_SCL_H_4;
    delay_s(10);//delay_us(1);

    while(TCS_SDA_READ_4)
    {
        t++;

        if(t > 250)
        {
            TCS34725_I2C_Stop_4();
            return 1;
        }
    }

    TCS_SCL_L_4;
    return 0;
}
/*********************************************/
//����ACKӦ��
void TCS34725_I2C_ACK()
{
    TCS_SCL_L;
    TCS_SDA_OUT();//sda�����
    TCS_SDA_L;
    delay_s(20);//delay_us(2);
    TCS_SCL_H;
    delay_s(20);//delay_us(2);
    TCS_SCL_L;
}
void TCS34725_I2C_ACK_1()
{
    TCS_SCL_L_1;
    TCS_SDA_OUT_1();//sda�����
    TCS_SDA_L_1;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_1;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_1;
}
void TCS34725_I2C_ACK_2()
{
    TCS_SCL_L_2;
    TCS_SDA_OUT_2();//sda�����
    TCS_SDA_L_2;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_2;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_2;
}
void TCS34725_I2C_ACK_3()
{
    TCS_SCL_L_3;
    TCS_SDA_OUT_3();//sda�����
    TCS_SDA_L_3;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_3;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_3;
}
void TCS34725_I2C_ACK_4()
{
    TCS_SCL_L_4;
    TCS_SDA_OUT_4();//sda�����
    TCS_SDA_L_4;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_4;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_4;
}
/*********************************************/
//������ACKӦ��
void TCS34725_I2C_NACK()
{
    TCS_SCL_L;
    TCS_SDA_OUT();//sda�����
    TCS_SDA_H;
    delay_s(20);//delay_us(2);
    TCS_SCL_H;
    delay_s(20);//delay_us(2);
    TCS_SCL_L;
}
void TCS34725_I2C_NACK_1()
{
    TCS_SCL_L_1;
    TCS_SDA_OUT_1();//sda�����
    TCS_SDA_H_1;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_1;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_1;
}
void TCS34725_I2C_NACK_2()
{
    TCS_SCL_L_2;
    TCS_SDA_OUT_2();//sda�����
    TCS_SDA_H_2;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_2;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_2;
}
void TCS34725_I2C_NACK_3()
{
    TCS_SCL_L_3;
    TCS_SDA_OUT_3();//sda�����
    TCS_SDA_H_3;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_3;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_3;
}
void TCS34725_I2C_NACK_4()
{
    TCS_SCL_L_4;
    TCS_SDA_OUT_4();//sda�����
    TCS_SDA_H_4;
    delay_s(20);//delay_us(2);
    TCS_SCL_H_4;
    delay_s(20);//delay_us(2);
    TCS_SCL_L_4;
}
/*********************************************/
//I2C����һ���ֽ�
void TCS34725_I2C_Send_Byte(u8 byte)
{
    u8 i;

    TCS_SDA_OUT();//sda�����
    TCS_SCL_L;//����ʱ�ӿ�ʼ���ݴ���

    for(i = 0; i < 8; i++)
    {
        if(((byte & 0x80) >> 7) == 1)TCS_SDA_H;
        else
            TCS_SDA_L;

        byte <<= 1;

        delay_s(20);//delay_us(2);
        TCS_SCL_H;
        delay_s(20);//delay_us(2);
        TCS_SCL_L;
        delay_s(20);//delay_us(2);
    }
}
void TCS34725_I2C_Send_Byte_1(u8 byte)
{
    u8 i;

    TCS_SDA_OUT_1();//sda�����
    TCS_SCL_L_1;//����ʱ�ӿ�ʼ���ݴ���

    for(i = 0; i < 8; i++)
    {
        if(((byte & 0x80) >> 7) == 1)TCS_SDA_H_1;
        else
            TCS_SDA_L_1;

        byte <<= 1;

        delay_s(20);//delay_us(2);
        TCS_SCL_H_1;
        delay_s(20);//delay_us(2);
        TCS_SCL_L_1;
        delay_s(20);//delay_us(2);
    }
}
void TCS34725_I2C_Send_Byte_2(u8 byte)
{
    u8 i;

    TCS_SDA_OUT_2();//sda�����
    TCS_SCL_L_2;//����ʱ�ӿ�ʼ���ݴ���

    for(i = 0; i < 8; i++)
    {
        if(((byte & 0x80) >> 7) == 1)TCS_SDA_H_2;
        else
            TCS_SDA_L_2;

        byte <<= 1;

        delay_s(20);//delay_us(2);
        TCS_SCL_H_2;
        delay_s(20);//delay_us(2);
        TCS_SCL_L_2;
        delay_s(20);//delay_us(2);
    }
}
void TCS34725_I2C_Send_Byte_3(u8 byte)
{
    u8 i;

    TCS_SDA_OUT_3();//sda�����
    TCS_SCL_L_3;//����ʱ�ӿ�ʼ���ݴ���

    for(i = 0; i < 8; i++)
    {
        if(((byte & 0x80) >> 7) == 1)TCS_SDA_H_3;
        else
            TCS_SDA_L_3;

        byte <<= 1;

        delay_s(20);//delay_us(2);
        TCS_SCL_H_3;
        delay_s(20);//delay_us(2);
        TCS_SCL_L_3;
        delay_s(20);//delay_us(2);
    }
}
void TCS34725_I2C_Send_Byte_4(u8 byte)
{
    u8 i;

    TCS_SDA_OUT_4();//sda�����
    TCS_SCL_L_4;//����ʱ�ӿ�ʼ���ݴ���

    for(i = 0; i < 8; i++)
    {
        if(((byte & 0x80) >> 7) == 1)TCS_SDA_H_4;
        else
            TCS_SDA_L_4;

        byte <<= 1;

        delay_s(20);//delay_us(2);
        TCS_SCL_H_4;
        delay_s(20);//delay_us(2);
        TCS_SCL_L_4;
        delay_s(20);//delay_us(2);
    }
}
/*********************************************/
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 TCS34725_I2C_Read_Byte(u8 ack)
{
    u8 i, receive = 0;

    TCS_SDA_IN();

    for(i = 0; i < 8; i++)
    {
        TCS_SCL_L;
        delay_s(20);//delay_us(2);
        TCS_SCL_H;
        receive <<= 1;

        if(TCS_SDA_READ) receive++;

        delay_s(10);//delay_us(1);
    }

    if (!ack) TCS34725_I2C_NACK();//����nACK
    else TCS34725_I2C_ACK(); //����ACK

    return receive;
}
u8 TCS34725_I2C_Read_Byte_1(u8 ack)
{
    u8 i, receive = 0;

    TCS_SDA_IN_1();

    for(i = 0; i < 8; i++)
    {
        TCS_SCL_L_1;
        delay_s(20);//delay_us(2);
        TCS_SCL_H_1;
        receive <<= 1;

        if(TCS_SDA_READ_1) receive++;

        delay_s(10);//delay_us(1);
    }

    if (!ack) TCS34725_I2C_NACK_1();//����nACK
    else TCS34725_I2C_ACK_1(); //����ACK

    return receive;
}
u8 TCS34725_I2C_Read_Byte_2(u8 ack)
{
    u8 i, receive = 0;

    TCS_SDA_IN_2();

    for(i = 0; i < 8; i++)
    {
        TCS_SCL_L_2;
        delay_s(20);//delay_us(2);
        TCS_SCL_H_2;
        receive <<= 1;

        if(TCS_SDA_READ_2) receive++;

        delay_s(10);//delay_us(1);
    }

    if (!ack) TCS34725_I2C_NACK_2();//����nACK
    else TCS34725_I2C_ACK_2(); //����ACK

    return receive;
}
u8 TCS34725_I2C_Read_Byte_3(u8 ack)
{
    u8 i, receive = 0;

    TCS_SDA_IN_3();

    for(i = 0; i < 8; i++)
    {
        TCS_SCL_L_3;
        delay_s(20);//delay_us(2);
        TCS_SCL_H_3;
        receive <<= 1;

        if(TCS_SDA_READ_3) receive++;

        delay_s(10);//delay_us(1);
    }

    if (!ack) TCS34725_I2C_NACK_3();//����nACK
    else TCS34725_I2C_ACK_3(); //����ACK

    return receive;
}
u8 TCS34725_I2C_Read_Byte_4(u8 ack)
{
    u8 i, receive = 0;

    TCS_SDA_IN_4();

    for(i = 0; i < 8; i++)
    {
        TCS_SCL_L_4;
        delay_s(20);//delay_us(2);
        TCS_SCL_H_4;
        receive <<= 1;

        if(TCS_SDA_READ_4) receive++;

        delay_s(10);//delay_us(1);
    }

    if (!ack) TCS34725_I2C_NACK_4();//����nACK
    else TCS34725_I2C_ACK_4(); //����ACK

    return receive;
}
/*********************************************/
/*******************************************************************************
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Write(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start();
    TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����
    TCS34725_I2C_Wait_ACK();

    for(i = 0; i < bytesNumber; i++)
    {
        TCS34725_I2C_Send_Byte(*(dataBuffer + i));
        TCS34725_I2C_Wait_ACK();
    }

    if(stopBit == 1) TCS34725_I2C_Stop();
}
void TCS34725_I2C_Write_1(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_1();
    TCS34725_I2C_Send_Byte_1((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����
    TCS34725_I2C_Wait_ACK_1();

    for(i = 0; i < bytesNumber; i++)
    {
        TCS34725_I2C_Send_Byte_1(*(dataBuffer + i));
        TCS34725_I2C_Wait_ACK_1();
    }

    if(stopBit == 1) TCS34725_I2C_Stop_1();
}
void TCS34725_I2C_Write_2(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_2();
    TCS34725_I2C_Send_Byte_2((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����
    TCS34725_I2C_Wait_ACK_2();

    for(i = 0; i < bytesNumber; i++)
    {
        TCS34725_I2C_Send_Byte_2(*(dataBuffer + i));
        TCS34725_I2C_Wait_ACK_2();
    }

    if(stopBit == 1) TCS34725_I2C_Stop_2();
}
void TCS34725_I2C_Write_3(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_3();
    TCS34725_I2C_Send_Byte_3((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����
    TCS34725_I2C_Wait_ACK_3();

    for(i = 0; i < bytesNumber; i++)
    {
        TCS34725_I2C_Send_Byte_3(*(dataBuffer + i));
        TCS34725_I2C_Wait_ACK_3();
    }

    if(stopBit == 1) TCS34725_I2C_Stop_3();
}
void TCS34725_I2C_Write_4(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_4();
    TCS34725_I2C_Send_Byte_4((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����
    TCS34725_I2C_Wait_ACK_4();

    for(i = 0; i < bytesNumber; i++)
    {
        TCS34725_I2C_Send_Byte_4(*(dataBuffer + i));
        TCS34725_I2C_Wait_ACK_4();
    }

    if(stopBit == 1) TCS34725_I2C_Stop_4();
}
/*******************************************************************************
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Read(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start();
    TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
    TCS34725_I2C_Wait_ACK();

    for(i = 0; i < bytesNumber; i++)
    {
        if(i == bytesNumber - 1)
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte(0);//��ȡ�����һ���ֽڷ���NACK
        }
        else
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte(1);
        }
    }

    if(stopBit == 1) TCS34725_I2C_Stop();
}
void TCS34725_I2C_Read_1(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_1();
    TCS34725_I2C_Send_Byte_1((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
    TCS34725_I2C_Wait_ACK_1();

    for(i = 0; i < bytesNumber; i++)
    {
        if(i == bytesNumber - 1)
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_1(0);//��ȡ�����һ���ֽڷ���NACK
        }
        else
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_1(1);
        }
    }

    if(stopBit == 1) TCS34725_I2C_Stop_1();
}
void TCS34725_I2C_Read_2(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_2();
    TCS34725_I2C_Send_Byte_2((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
    TCS34725_I2C_Wait_ACK_2();

    for(i = 0; i < bytesNumber; i++)
    {
        if(i == bytesNumber - 1)
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_2(0);//��ȡ�����һ���ֽڷ���NACK
        }
        else
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_2(1);
        }
    }

    if(stopBit == 1) TCS34725_I2C_Stop_2();
}
void TCS34725_I2C_Read_3(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_3();
    TCS34725_I2C_Send_Byte_3((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
    TCS34725_I2C_Wait_ACK_3();

    for(i = 0; i < bytesNumber; i++)
    {
        if(i == bytesNumber - 1)
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_3(0);//��ȡ�����һ���ֽڷ���NACK
        }
        else
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_3(1);
        }
    }

    if(stopBit == 1) TCS34725_I2C_Stop_3();
}
void TCS34725_I2C_Read_4(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
    u8 i = 0;

    TCS34725_I2C_Start_4();
    TCS34725_I2C_Send_Byte_4((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
    TCS34725_I2C_Wait_ACK_4();

    for(i = 0; i < bytesNumber; i++)
    {
        if(i == bytesNumber - 1)
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_4(0);//��ȡ�����һ���ֽڷ���NACK
        }
        else
        {
            *(dataBuffer + i) = TCS34725_I2C_Read_Byte_4(1);
        }
    }

    if(stopBit == 1) TCS34725_I2C_Stop_4();
}
/*******************************************************************************
 * @brief Writes data into TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes that will be sent.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Write(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;

    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }

    TCS34725_I2C_Write(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
void TCS34725_Write_1(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;

    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }

    TCS34725_I2C_Write_1(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
void TCS34725_Write_2(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;

    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }

    TCS34725_I2C_Write_2(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
void TCS34725_Write_3(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;

    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }

    TCS34725_I2C_Write_3(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
void TCS34725_Write_4(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;

    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }

    TCS34725_I2C_Write_4(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
/*******************************************************************************
 * @brief Reads data from TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Read(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    TCS34725_I2C_Write(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
    TCS34725_I2C_Read(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
void TCS34725_Read_1(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    TCS34725_I2C_Write_1(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
    TCS34725_I2C_Read_1(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
void TCS34725_Read_2(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    TCS34725_I2C_Write_2(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
    TCS34725_I2C_Read_2(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
void TCS34725_Read_3(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    TCS34725_I2C_Write_3(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
    TCS34725_I2C_Read_3(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
void TCS34725_Read_4(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    TCS34725_I2C_Write_4(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
    TCS34725_I2C_Read_4(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
/*******************************************************************************
 * @brief TCS34725���û���ʱ��
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(u8 time)
{
    TCS34725_Write(TCS34725_ATIME, &time, 1);
}
void TCS34725_SetIntegrationTime_1(u8 time)
{
    TCS34725_Write_1(TCS34725_ATIME, &time, 1);
}
void TCS34725_SetIntegrationTime_2(u8 time)
{
    TCS34725_Write_2(TCS34725_ATIME, &time, 1);
}
void TCS34725_SetIntegrationTime_3(u8 time)
{
    TCS34725_Write_3(TCS34725_ATIME, &time, 1);
}
void TCS34725_SetIntegrationTime_4(u8 time)
{
    TCS34725_Write_4(TCS34725_ATIME, &time, 1);
}
/*******************************************************************************
 * @brief TCS34725��������
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(u8 gain)
{
    TCS34725_Write(TCS34725_CONTROL, &gain, 1);
}
void TCS34725_SetGain_1(u8 gain)
{
    TCS34725_Write_1(TCS34725_CONTROL, &gain, 1);
}
void TCS34725_SetGain_2(u8 gain)
{
    TCS34725_Write_1(TCS34725_CONTROL, &gain, 1);
}
void TCS34725_SetGain_3(u8 gain)
{
    TCS34725_Write_3(TCS34725_CONTROL, &gain, 1);
}
void TCS34725_SetGain_4(u8 gain)
{
    TCS34725_Write_4(TCS34725_CONTROL, &gain, 1);
}
/*******************************************************************************
 * @brief TCS34725ʹ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Enable(void)
{
    u8 cmd = TCS34725_ENABLE_PON;

    TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
    //delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
void TCS34725_Enable_1(void)
{
    u8 cmd = TCS34725_ENABLE_PON;

    TCS34725_Write_1(TCS34725_ENABLE, &cmd, 1);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write_1(TCS34725_ENABLE, &cmd, 1);
    //delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
void TCS34725_Enable_2(void)
{
    u8 cmd = TCS34725_ENABLE_PON;

    TCS34725_Write_2(TCS34725_ENABLE, &cmd, 1);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write_2(TCS34725_ENABLE, &cmd, 1);
    //delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
void TCS34725_Enable_3(void)
{
    u8 cmd = TCS34725_ENABLE_PON;

    TCS34725_Write_3(TCS34725_ENABLE, &cmd, 1);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write_3(TCS34725_ENABLE, &cmd, 1);
    //delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
void TCS34725_Enable_4(void)
{
    u8 cmd = TCS34725_ENABLE_PON;

    TCS34725_Write_4(TCS34725_ENABLE, &cmd, 1);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write_4(TCS34725_ENABLE, &cmd, 1);
    //delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
/*******************************************************************************
 * @brief TCS34725ʧ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Disable(void)
{
    u8 cmd = 0;

    TCS34725_Read(TCS34725_ENABLE, &cmd, 1);
    cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
}
void TCS34725_Disable_1(void)
{
    u8 cmd = 0;

    TCS34725_Read_1(TCS34725_ENABLE, &cmd, 1);
    cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_Write_1(TCS34725_ENABLE, &cmd, 1);
}
void TCS34725_Disable_2(void)
{
    u8 cmd = 0;

    TCS34725_Read_2(TCS34725_ENABLE, &cmd, 1);
    cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_Write_2(TCS34725_ENABLE, &cmd, 1);
}
void TCS34725_Disable_3(void)
{
    u8 cmd = 0;

    TCS34725_Read_3(TCS34725_ENABLE, &cmd, 1);
    cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_Write_3(TCS34725_ENABLE, &cmd, 1);
}
void TCS34725_Disable_4(void)
{
    u8 cmd = 0;

    TCS34725_Read_4(TCS34725_ENABLE, &cmd, 1);
    cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_Write_4(TCS34725_ENABLE, &cmd, 1);
}
/*******************************************************************************
 * @brief TCS34725��ʼ��
 *
 * @return ID - ID�Ĵ����е�ֵ
*******************************************************************************/
u8 TCS34725_Init(void)
{
    u8 id = 0;

    TCS34725_I2C_Init();
    TCS34725_Read(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;

    if(id == 0x4D | id == 0x44)
    {
        TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
        TCS34725_SetGain(TCS34725_GAIN_1X);
        TCS34725_Enable();
        return 1;
    }

    return 0;
}
u8 TCS34725_Init_1(void)
{
    u8 id = 0;

    TCS34725_I2C_Init_1();
    TCS34725_Read_1(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;

    if(id == 0x4D | id == 0x44)
    {
        TCS34725_SetIntegrationTime_1(TCS34725_INTEGRATIONTIME_50MS);
        TCS34725_SetGain_1(TCS34725_GAIN_1X);
        TCS34725_Enable_1();
        return 1;
    }

    return 0;
}
u8 TCS34725_Init_2(void)
{
    u8 id = 0;

    TCS34725_I2C_Init_2();
    TCS34725_Read_2(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;

    if(id == 0x4D | id == 0x44)
    {
        TCS34725_SetIntegrationTime_2(TCS34725_INTEGRATIONTIME_50MS);
        TCS34725_SetGain_2(TCS34725_GAIN_1X);
        TCS34725_Enable_2();
        return 1;
    }

    return 0;
}
u8 TCS34725_Init_3(void)
{
    u8 id = 0;

    TCS34725_I2C_Init_3();
    TCS34725_Read_3(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;

    if(id == 0x4D | id == 0x44)
    {
        TCS34725_SetIntegrationTime_3(TCS34725_INTEGRATIONTIME_50MS);
        TCS34725_SetGain_3(TCS34725_GAIN_1X);
        TCS34725_Enable_3();
        return 1;
    }

    return 0;
}
u8 TCS34725_Init_4(void)
{
    u8 id = 0;

    TCS34725_I2C_Init_4();
    TCS34725_Read_4(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;

    if(id == 0x4D | id == 0x44)
    {
        TCS34725_SetIntegrationTime_4(TCS34725_INTEGRATIONTIME_50MS);
        TCS34725_SetGain_4(TCS34725_GAIN_1X);
        TCS34725_Enable_4();
        return 1;
    }

    return 0;
}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return data - ��ͨ����ת��ֵ
*******************************************************************************/
u16 TCS34725_GetChannelData(u8 reg)
{
    u8 tmp[2] = {0, 0};
    u16 data;

    TCS34725_Read(reg, tmp, 2);
    data = (tmp[1] << 8) | tmp[0];

    return data;
}
u16 TCS34725_GetChannelData_1(u8 reg)
{
    u8 tmp[2] = {0, 0};
    u16 data;

    TCS34725_Read_1(reg, tmp, 2);
    data = (tmp[1] << 8) | tmp[0];

    return data;
}
u16 TCS34725_GetChannelData_2(u8 reg)
{
    u8 tmp[2] = {0, 0};
    u16 data;

    TCS34725_Read_2(reg, tmp, 2);
    data = (tmp[1] << 8) | tmp[0];

    return data;
}
u16 TCS34725_GetChannelData_3(u8 reg)
{
    u8 tmp[2] = {0, 0};
    u16 data;

    TCS34725_Read_3(reg, tmp, 2);
    data = (tmp[1] << 8) | tmp[0];

    return data;
}
u16 TCS34725_GetChannelData_4(u8 reg)
{
    u8 tmp[2] = {0, 0};
    u16 data;

    TCS34725_Read_4(reg, tmp, 2);
    data = (tmp[1] << 8) | tmp[0];

    return data;
}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return 1 - ת����ɣ����ݿ���
 *   	   0 - ת��δ��ɣ����ݲ�����
*******************************************************************************/
u8 TCS34725_GetRawData(COLOR_RGBC *rgbc)
{
    u8 status = TCS34725_STATUS_AVALID;

    TCS34725_Read(TCS34725_STATUS, &status, 1);

    if(status & TCS34725_STATUS_AVALID)
    {
        rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);
        rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);
        rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);
        rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
        return 1;
    }

    return 0;
}
u8 TCS34725_GetRawData_1(COLOR_RGBC *rgbc)
{
    u8 status = TCS34725_STATUS_AVALID;

    TCS34725_Read_1(TCS34725_STATUS, &status, 1);

    if(status & TCS34725_STATUS_AVALID)
    {
        rgbc->c = TCS34725_GetChannelData_1(TCS34725_CDATAL);
        rgbc->r = TCS34725_GetChannelData_1(TCS34725_RDATAL);
        rgbc->g = TCS34725_GetChannelData_1(TCS34725_GDATAL);
        rgbc->b = TCS34725_GetChannelData_1(TCS34725_BDATAL);
        return 1;
    }

    return 0;
}
u8 TCS34725_GetRawData_2(COLOR_RGBC *rgbc)
{
    u8 status = TCS34725_STATUS_AVALID;

    TCS34725_Read_2(TCS34725_STATUS, &status, 1);

    if(status & TCS34725_STATUS_AVALID)
    {
        rgbc->c = TCS34725_GetChannelData_2(TCS34725_CDATAL);
        rgbc->r = TCS34725_GetChannelData_2(TCS34725_RDATAL);
        rgbc->g = TCS34725_GetChannelData_2(TCS34725_GDATAL);
        rgbc->b = TCS34725_GetChannelData_2(TCS34725_BDATAL);
        return 1;
    }

    return 0;
}
u8 TCS34725_GetRawData_3(COLOR_RGBC *rgbc)
{
    u8 status = TCS34725_STATUS_AVALID;

    TCS34725_Read_3(TCS34725_STATUS, &status, 1);

    if(status & TCS34725_STATUS_AVALID)
    {
        rgbc->c = TCS34725_GetChannelData_3(TCS34725_CDATAL);
        rgbc->r = TCS34725_GetChannelData_3(TCS34725_RDATAL);
        rgbc->g = TCS34725_GetChannelData_3(TCS34725_GDATAL);
        rgbc->b = TCS34725_GetChannelData_3(TCS34725_BDATAL);
        return 1;
    }

    return 0;
}
u8 TCS34725_GetRawData_4(COLOR_RGBC *rgbc)
{
    u8 status = TCS34725_STATUS_AVALID;

    TCS34725_Read_4(TCS34725_STATUS, &status, 1);

    if(status & TCS34725_STATUS_AVALID)
    {
        rgbc->c = TCS34725_GetChannelData_4(TCS34725_CDATAL);
        rgbc->r = TCS34725_GetChannelData_4(TCS34725_RDATAL);
        rgbc->g = TCS34725_GetChannelData_4(TCS34725_GDATAL);
        rgbc->b = TCS34725_GetChannelData_4(TCS34725_BDATAL);
        return 1;
    }

    return 0;
}
/******************************************************************************/
//RGBתHSL
void RGBtoHSL(COLOR_RGBC *Rgb, COLOR_HSL *Hsl)
{
    u8 maxVal, minVal, difVal;
    u8 r = Rgb->r * 100 / Rgb->c; //[0-100]
    u8 g = Rgb->g * 100 / Rgb->c;
    u8 b = Rgb->b * 100 / Rgb->c;

    maxVal = max3v(r, g, b);
    minVal = min3v(r, g, b);
    difVal = maxVal - minVal;

    //��������
    Hsl->l = (maxVal + minVal) / 2; //[0-100]

    if(maxVal == minVal)//��r=g=b,�Ҷ�
    {
        Hsl->h = 0;
        Hsl->s = 0;
    }
    else
    {
        //����ɫ��
        if(maxVal == r)
        {
            if(g >= b)
                Hsl->h = 60 * (g - b) / difVal;
            else
                Hsl->h = 60 * (g - b) / difVal + 360;
        }
        else
        {
            if(maxVal == g)Hsl->h = 60 * (b - r) / difVal + 120;
            else if(maxVal == b)Hsl->h = 60 * (r - g) / difVal + 240;
        }

        //���㱥�Ͷ�
        if(Hsl->l <= 50)Hsl->s = difVal * 100 / (maxVal + minVal); //[0-100]
        else
            Hsl->s = difVal * 100 / (200 - (maxVal + minVal));
    }
}
/******************************************************************************/


