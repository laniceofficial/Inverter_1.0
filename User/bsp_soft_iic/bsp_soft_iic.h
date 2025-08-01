#ifndef _BSP_SOFT_IIC_H

#define _BSP_SOFT_I2C_H
#include "gpio.h"

//每次使用不同的IO时需要在此更改引脚和延时时间，目前使用的时SCL-PA15，SDA—PB7
// scl的端口
#define SI2C_SCL_PORT   GPIOA
#define SI2C_SCL_PIN    GPIO_PIN_15
// sda的端口
#define SI2C_SDA_PORT   GPIOB
#define SI2C_SDA_PIN    GPIO_PIN_7
#define WAIT_TIME 80

// void SI2C_Delay(void);
void SI2C_W_SCL(unsigned char x); // 控制scl电平
void SI2C_W_SDA(unsigned char x); // 控制sda电平
unsigned char SI2C_R_SDA(void);   // 读取sda电平
void SI2C_Start(void);
void SI2C_Stop(void);
void SI2C_SendByte(uint8_t Byte);
uint8_t SI2C_ReadByte(void);
uint8_t SI2C_WaitAck(void);
void SI2C_Ack(void);
void SI2C_NAck(void);
void SI2C_init(void);
void SI2C_Delay_init(void);
void SSI2C_Delay_us(uint32_t nus);

#endif /*_BSP_SOFT_IIC_H*/
