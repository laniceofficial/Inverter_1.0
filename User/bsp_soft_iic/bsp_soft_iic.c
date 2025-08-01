#include "bsp_soft_iic.h"
// #include "main.h"
/*函 数 名 : SI2C_Delay
 *功能说明 : I2C总线位延迟，最快400KHz;参考野火
 *形 参：无
 *返 回 值 : 无*/
static uint16_t WAIT_TIME_ = 100; // 延时循环次数，越小SCL频率越高，最大400KHz
static void SI2C_Delay(void)
{
    uint8_t i;

    /*下面的时间是通过逻辑分析仪测试得到的。
    工作条件：CPU主频72MHz ，MDK编译环境，1级优化
    循环次数为10时，SCL频率 = 205KHz
    循环次数为7时，SCL频率 = 347KHz， SCL高电平时间1.5us，SCL低电平时间2.87us
    循环次数为5时，SCL频率 = 421KHz， SCL高电平时间1.25us，SCL低电平时间2.375us
    */
    for (i = 0; i < WAIT_TIME_; i++)
        ;
}
void SI2C_W_SCL(unsigned char x) // 控制scl电平
{

    HAL_GPIO_WritePin(SI2C_SCL_PORT, SI2C_SCL_PIN, (GPIO_PinState)x);
    SI2C_Delay();
}

void SI2C_W_SDA(unsigned char x) // 控制sda电平
{

    HAL_GPIO_WritePin(SI2C_SDA_PORT, SI2C_SDA_PIN, (GPIO_PinState)x);
    SI2C_Delay();
}
unsigned char SI2C_R_SDA(void) // 读取sda电平
{
    unsigned char u8;
    u8 = HAL_GPIO_ReadPin(SI2C_SCL_PORT, SI2C_SCL_PIN);
    SI2C_Delay();
    return u8;
}

/**
 * 函    数：I2C起始
 * 参    数：无
 * 返 回 值：无
 */
void SI2C_Start(void)
{
    SI2C_W_SDA(1); // 一定要先拉高SDA再拉高SCL
    SI2C_W_SCL(1);

    SI2C_Delay();
    SI2C_W_SDA(0); // 在SCL拉高的时候拉低SDA
    SI2C_Delay();
    SI2C_W_SCL(0); // 随后拉低SCL，即为了占用总线，也为了方便总线时序的拼接
    SI2C_Delay();
}

/**
 * 函    数：I2C终止
 * 参    数：无
 * 返 回 值：无
 */
void SI2C_Stop(void)
{
    SI2C_W_SDA(0);  // 发送结束条件的数据信号
    SI2C_W_SCL(1);
    SI2C_Delay(); // 结束条件建立时间大于4μ
    SI2C_W_SDA(1); // 在SCL高电平期间，释放SDA，产生终止信号
    
}

/*******************************************************************************
 * 函 数 名         : i2c_send
 * 函数功能         : iic发送数据
 * 输    入         : uint8_t dat,要发送的数据
 * 输    出         : 无
 *******************************************************************************/
void SI2C_SendByte(uint8_t Byte)
{
    uint8_t i;

    /*循环8次，主机依次发送数据的每一位*/
    for (i = 0; i < 8; i++)
    {
        /*使用掩码的方式取出Byte的指定一位数据并写入到SDA线*/
        /*两个!的作用是，让所有非零的值变为1*/
        SI2C_W_SDA(!!(Byte & (0x80 >> i)));
        SI2C_W_SCL(1); // 释放SCL，从机在SCL高电平期间读取SDA
        SI2C_W_SCL(0); // 拉低SCL，主机开始发送下一位数据
    }

    SI2C_W_SCL(1); // 额外的一个时钟，不处理应答信号
    SI2C_W_SCL(0);
}

/*
**********************************************
* 函 数 名: i2c_ReadByte
* 功能说明: CPU从I2C总线设备读取8bit数据
* 形    参：无
* 返 回 值: 读到的数据
**********************************************
*/
uint8_t SI2C_ReadByte(void)
{
    uint8_t i;
    uint8_t value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        SI2C_W_SCL(1);
        SI2C_Delay();
        if (SI2C_R_SDA())
        {
            value++;
        }
        SI2C_W_SCL(1);
        SI2C_Delay();
    }
    return value;
}

/*
*************************************************
* 函 数 名: i2c_WaitAck
* 功能说明:
CPU产生一个时钟，并读取器件的ACK应答信号
* 形    参：无
* 返 回 值: 返回0表示正确应答，1表示无器件响应
*************************************************
*/
uint8_t SI2C_WaitAck(void)
{
    uint8_t re;

    SI2C_W_SDA(1); /* CPU释放SDA总线 */
    SI2C_Delay();
    SI2C_W_SCL(1); /* CPU驱动SCL = 1,
                    此时器件会返回ACK应答 */
    SI2C_Delay();
    if (SI2C_R_SDA())
    { /* CPU读取SDA口线状态 */
        re = 1;
    }
    else
    {
        re = 0;
    }
    SI2C_W_SCL(0);
    SI2C_Delay();
    return re;
}

/*
************************************************
* 函 数 名: i2c_Ack
* 功能说明: CPU产生一个ACK信号
* 形    参：无
* 返 回 值: 无
************************************************
*/
void SI2C_Ack(void)
{
    SI2C_W_SDA(0); /* CPU驱动SDA = 0 */
    SI2C_Delay();
    SI2C_W_SCL(1); /* CPU产生1个时钟 */
    SI2C_Delay();   // 此过程,SDA持续低电平，为应答
    SI2C_W_SCL(0);
    SI2C_Delay();
    SI2C_W_SDA(1); /* CPU释放SDA总线 */
}

/*
**********************************************
* 函 数 名: i2c_NAck
* 功能说明: CPU产生1个NACK信号
* 形    参：无
* 返 回 值: 无
**********************************************
*/
void SI2C_NAck(void)
{
    SI2C_W_SDA(1); /* CPU驱动SDA = 1 */
    SI2C_Delay();
    SI2C_W_SCL(1);       /* CPU产生1个时钟 */
    SI2C_Delay();        // 此过程,SDA持续低电平，为应答
    SI2C_W_SCL(0);
    SI2C_Delay();
}

void SI2C_init(void)
{
    SI2C_Delay_init();

}

/**********************************************************/
static uint32_t g_fac_us = 0; /* us 延时倍乘数 */
/**
 * @brief    初始化延时函数
 * @param    无
 * @retval   无
 */
void SI2C_Delay_init(void)
{
    g_fac_us = HAL_RCC_GetHCLKFreq() / 1000000; // 获取MCU的主频
}

/**
 * @brief    正点原子us延时函数需要设定g_fac_us
 * @note     使用时钟摘取法来做us延时
 * @param    nus:要延时的us数
 * @note     nus取值范围：0 ~ (2^32 / fac_us)(fac_us一般等于系统主频)
 * @retval   无
 */
void SSI2C_Delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; /*LOAD的值*/
    ticks = nus * g_fac_us;          /*需要的节拍数*/

    told = SysTick->VAL; /*刚进入时的计数器值*/
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow; /*注意一下SYSTICK是一个递减的计数器*/
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break; /*时间超过/等于要延时的时间，则退出*/
            }
        }
    }
}
