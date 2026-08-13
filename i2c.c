#include "i2c.h"
#include "delay.h"

void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SDA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDA1_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = SCL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCL1_GPIO_Port, &GPIO_InitStruct);
}

void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = SDA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDA1_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = SCL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCL1_GPIO_Port, &GPIO_InitStruct);
}

// 发送IIC起始信号
bool I2C2_Start(void)
{
	SDA_OUT();
	Pin_SCL_H; // 拉高时钟线
	Pin_SDA_H; // 拉高信号线
	Delay_us(t_i2c);
	SDA_IN();
	if(!Read_SDA_Pin)		return false;
	SDA_OUT();
	Pin_SDA_L;
	Delay_us(t_i2c);
	Pin_SDA_L;
	Delay_us(t_i2c);
	return true;
}
 
// 发送IIC停止信号
bool I2C2_Stop(void)
{
	SDA_OUT();
	Pin_SCL_H;
	Pin_SDA_L;
	Delay_us(t_i2c);
	SDA_IN();
	if(Read_SDA_Pin)	return false;
	SDA_OUT();
	Pin_SDA_H;
	Delay_us(t_i2c);
	SDA_IN();
	if(!Read_SDA_Pin) return false;
	SDA_OUT();
	Pin_SDA_H;
	Delay_us(t_i2c);
	return true;
}
 
// IIC等待ACK信号
uint8_t I2C2_Wait_Ack(void)
{
	SDA_OUT();
	Pin_SCL_L;
	Delay_us(t_i2c);
	Pin_SDA_H;
	Pin_SCL_H;
	Delay_us(t_i2c);
	SDA_IN();
	if(Read_SDA_Pin)
	{
		Pin_SCL_L;
		Delay_us(t_i2c);
		return false;
	}
	Pin_SCL_L;
	Delay_us(t_i2c);
	return true;
}
 
// IIC发送一个字节
void I2C2_Send_Byte(uint8_t txd)
{
	SDA_OUT();
	u16 i;
	for(i=0; i<8; i++)
	{
		Pin_SCL_L;
		Delay_us(t_i2c);
		if(txd & 0x80)Pin_SDA_H;
		else Pin_SDA_L;
		txd <<= 1;
		Pin_SCL_H;
		Delay_us(t_i2c);
	}
}
 
// IIC不发送ACK信号（读字节结束后由主机回 NACK，通知从机停止发送）
void I2C2_NAck(void)
{
	SDA_OUT();
	Pin_SCL_L;
	Delay_us(t_i2c);
	Pin_SDA_H;
	Pin_SCL_H;
	Delay_us(t_i2c);
	Pin_SCL_L;
	Delay_us(t_i2c);
}
 
// IIC读取一个字节
uint8_t I2C2_Read_Byte(void)
{
	SDA_IN();
	uint8_t rxd = 0;
	u16 i;
	for(i=0; i<8; i++)
	{
		rxd <<= 1;
		Pin_SCL_L;
		Delay_us(t_i2c);
		Pin_SCL_H;	
		Delay_us(t_i2c);
		if(Read_SDA_Pin)
		{
			rxd |= 0x01;
		}
	}
	return rxd;
}
 
// 向从机指定寄存器写一个字节（驱动底层入口：MS5351/2 全部写入走这里）
bool my_I2C_sendREG(uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C2_Start())		return false;
    I2C2_Send_Byte(0xC0);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_Address);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_data);
    if(!I2C2_Wait_Ack()) { I2C2_Stop(); return false;	}
    if(!I2C2_Stop()) return false;
    return true;
}

// 读从机指定寄存器一个字节（0xC0 写地址 / 0xC1 读地址，MS5351/2 固定从机地址）
uint8_t my_I2C2_Read_REG(uint8_t REG_Address)
{
    uint8_t data;
    if(!I2C2_Start())	return false;
    I2C2_Send_Byte(0xC0);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    I2C2_Send_Byte(REG_Address);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    if(!I2C2_Start())	return false;
    I2C2_Send_Byte(0xC1);
    if(!I2C2_Wait_Ack()) { I2C2_Stop();	return false;	}
    data = I2C2_Read_Byte();
    I2C2_NAck();
    if(!I2C2_Stop())	return false;	
    return data;
}
