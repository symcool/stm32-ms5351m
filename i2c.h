#ifndef __I2C_H
#define	__I2C_H

#include "sys.h"
#include "rf.h"

typedef enum {false = 0, true = !false} bool;

#define Pin_SCL_L		SCL1_OUT=0
#define Pin_SCL_H		SCL1_OUT=1
 
#define Pin_SDA_L		SDA1_OUT=0
#define Pin_SDA_H		SDA1_OUT=1
 
#define Read_SDA_Pin	SDA1_IN

#define	t_i2c 1

void SDA_OUT(void);
void SDA_IN(void);

bool I2C2_Start(void);
bool I2C2_Stop(void);
void I2C2_Send_Byte(uint8_t txd);
uint8_t I2C2_Wait_Ack(void);
void I2C2_NAck(void);
uint8_t I2C2_Read_Byte(void);

bool my_I2C_sendREG(uint8_t REG_Address,uint8_t REG_data);
uint8_t my_I2C2_Read_REG(uint8_t REG_Address);
#endif
