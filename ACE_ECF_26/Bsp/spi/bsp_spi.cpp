/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_spi.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/9/20
 * @brief   SPI的BSP层文件
 ******************************************************************************
 * @verbatim
 * 使用方法：
    // 1、创建一个SPI_c对象，需要传入一个SPI_InitConfig_s结构体和用户自己定义的回调函数，用于初始化SPI实例
    void spi_callback(SPI_c* register_instance)
    {
        
    }
    SPI_InitConfig_s spi_1 = 
    {
        .spi_handle = &hspi1,
        .GPIOx = GPIOA,       // CS引脚配置详细请看.md文件
        .cs_pin = GPIO_PIN_1,  
        .spi_work_mode = IT,
    };
    uint8_t data[5]={0,0,0,0,0};
    SPI_c spi_1_instance(spi_1, spi_callback);   
 * @attention
 *      1、如果你没有在CubeMX中为spi分配dma通道，请不要使用dma模式
 *      2、需要用一根片选 CS 去控制两个或两个以上的 SPI 从设备时，需要在cubemx里面设置对应的GPIO。
 *      3、初始化传入参数config中的GPIOx（GPIOA，GPIOB，...）和cs_pin（GPIO_PIN_1,GPIO_PIN_2, ...）都是HAL库内建的宏，
 *         在CubeMX初始化的时候若有给gpio分配标签则填入对应名字即可，否则填入原本的宏.
 *      更多详细有关SPI的知识点请看.md文件
 * @version           time
 * v1.0   基础版本    2024-9-20   目前版本还未测试，目前找不到合适的用bsp_spi的设备
 ************************** Dongguan-University of Technology -ACE***************************/
// 本文件头文件
#include "bsp_spi.hpp"
// C++语言库文件
#include "stdlib.h"
namespace BSP_n
{


// 初始化CAN实例指针数组
SPI_c *SPI_c::spi_instance_[MX_SPI_BUS_SLAVE_CNT] = {nullptr};
// 初始化CAN实例指针数组下标
uint8_t SPI_c::idx_ = 0; 
uint8_t SPI_c::SPIDeviceOnGoing[SPI_DEVICE_CNT] = {1}; // 用于判断当前spi是否正在传输,防止多个模块同时使用一个spi总线 (0: 正在传输, 1: 未传输)

SPI_c::SPI_c(SPI_InitConfig_s config)
{
    Init(config.spi_handle,
         config.GPIOx,
         config.cs_pin,
         config.spi_work_mode,
         config.spi_module_callback);
}
void SPI_c::Init(SPI_InitConfig_s config)
{
    Init(config.spi_handle,
         config.GPIOx,
         config.cs_pin,
         config.spi_work_mode,
         config.spi_module_callback);
}
void SPI_c::Init(SPI_HandleTypeDef *spi_handle,
               GPIO_TypeDef *GPIOx,
               uint16_t cs_pin,
               TransType_t spi_work_mode,
               void (*spi_module_callback)(SPI_c *register_instance))
{
    if (idx_ >= MX_SPI_BUS_SLAVE_CNT) // 超过最大实例数
    {
        while (true)
        {
        }
    }
    this->spi_handle_ = spi_handle;
    this->GPIOx_ = GPIOx;
    this->cs_pin_ = cs_pin;
    this->spi_work_mode_ = spi_work_mode;
    if (this->spi_handle_->Instance == SPI1)
    {
        this->CS_State_ = SPIDeviceOnGoing[0];
    }
    else if (this->spi_handle_->Instance == SPI2)
    {
        this->CS_State_ = SPIDeviceOnGoing[1];
    }
    else
    {
        while (true)
        {
        }
    }
    spi_instance_[idx_++] = this;
}


/**
 * @brief 通过spi向对应从机发送数据
 * @todo  后续考虑加入阻塞模式下的timeout参数
 *
 * @param ptr_data 要发送的数据
 * @param len 待发送的数据长度
 */
void SPI_c::Transmit(uint8_t *ptr_data, uint8_t len)
{
    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_RESET);
    switch (this->spi_work_mode_)
    {
    case TransType_t::DMA:
        HAL_SPI_Transmit_DMA(this->spi_handle_, ptr_data, len);
        break;
    case TransType_t::IT:
        HAL_SPI_Transmit_IT(this->spi_handle_, ptr_data, len);
        break;
    case TransType_t::BLOCK:
        HAL_SPI_Transmit(this->spi_handle_, ptr_data, len, 1000); // 默认50ms超时
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_SET);
        break;
    default:
        while (true)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        break;
    }
}

/**
 * @brief 通过spi从从机获取数据
 * @attention 特别注意:请保证ptr_data在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
 * 
 * @param ptr_data 接受数据buffer的首地址
 * @param len 待接收的长度
 */
void SPI_c::Receive(uint8_t *ptr_data, uint8_t len)
{
    // 用于稍后回调使用
    this->rx_size_ = len;
    this->rx_buffer_ = ptr_data;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_RESET);
    switch (this->spi_work_mode_)
    {
    case TransType_t::DMA:
        HAL_SPI_Receive_DMA(this->spi_handle_, ptr_data, len);
        break;
    case TransType_t::IT:
        HAL_SPI_Receive_IT(this->spi_handle_, ptr_data, len);
        break;
    case TransType_t::BLOCK:
        HAL_SPI_Receive(this->spi_handle_, ptr_data, len, 1000);
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        break;
    }
}

/**
 * @brief 通过spi利用移位寄存器同时收发数据
 * @todo  后续加入阻塞模式下的timeout参数
 * @attention 特别注意:请保证ptr_data_rx在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
 * 
 * @param ptr_data_rx 接收数据地址
 * @param ptr_data_tx 发送数据地址
 * @param len 接收&发送的长度
 */
void SPI_c::TransRecv(uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    // 用于稍后回调使用,请保证ptr_data_rx在回调函数被调用之前仍然在作用域内,否则析构之后的行为是未定义的!!!
    this->rx_size_ = len;
    this->rx_buffer_ = ptr_data_rx;
    // 等待上一次传输完成
    if (this->spi_handle_->Instance == SPI1)
    {
        while (!SPIDeviceOnGoing[0])
        {
        };
    }
    else if (this->spi_handle_->Instance == SPI2)
    {
        while (!SPIDeviceOnGoing[1])
        {
        };
    }
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_RESET);
    this->CS_State_ = HAL_GPIO_ReadPin(this->GPIOx_, this->cs_pin_);
    switch (this->spi_work_mode_)
    {
    case TransType_t::DMA:
        HAL_SPI_TransmitReceive_DMA(this->spi_handle_, ptr_data_tx, ptr_data_rx, len);
        break;
    case TransType_t::IT:
        HAL_SPI_TransmitReceive_IT(this->spi_handle_, ptr_data_tx, ptr_data_rx, len);
        break;
    case TransType_t::BLOCK:
        HAL_SPI_TransmitReceive(this->spi_handle_, ptr_data_tx, ptr_data_rx, len, 1000); // 默认50ms超时
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(this->GPIOx_, this->cs_pin_, GPIO_PIN_SET);
        this->CS_State_ = HAL_GPIO_ReadPin(this->GPIOx_, this->cs_pin_);
        break;
    default:
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况
        break;
    }
}

/**
 * @brief 设定spi收发的工作模式
 *
 * @param spi_ins spi实例指针
 * @param spi_mode 工作模式,包括阻塞模式(TransType_t::BLOCK),中断模式(IT),DMA模式.详见TransType_t的定义
 * 
 * @todo 是否直接将mode作为transmit/recv的参数,而不是作为spi实例的属性?两者各有优劣
 */
void SPI_c::SetMode(TransType_t spi_mode)
{
    if (spi_mode != TransType_t::DMA && spi_mode != TransType_t::IT && spi_mode != TransType_t::BLOCK)
        while (true)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界导致模式被异常修改的情况

    if (this->spi_work_mode_ != spi_mode)
    {
        this->spi_work_mode_ = spi_mode;
    }
}

void SPI_c::HandleTxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (size_t i = 0; i < idx_; i++)
    {
        // 如果是当前spi硬件发出的complete,且cs_pin为低电平(说明正在传输),则尝试调用回调函数
        if (spi_instance_[i]->spi_handle_ == hspi && // 显然同一时间一条总线只能有一个从机在接收数据
            HAL_GPIO_ReadPin(spi_instance_[i]->GPIOx_, spi_instance_[i]->cs_pin_) == GPIO_PIN_RESET)
        {
            // 先拉高片选,结束传输,在判断是否有回调函数,如果有则调用回调函数
            HAL_GPIO_WritePin(spi_instance_[i]->GPIOx_, spi_instance_[i]->cs_pin_, GPIO_PIN_SET);
            spi_instance_[i]->CS_State_ = HAL_GPIO_ReadPin(spi_instance_[i]->GPIOx_, spi_instance_[i]->cs_pin_);
            // @todo 后续添加holdon模式,由用户自行决定何时释放片选,允许进行连续传输
            if (spi_instance_[i]->spi_module_callback != NULL) // 回调函数不为空, 则调用回调函数
                spi_instance_[i]->spi_module_callback(spi_instance_[i]);
            return;
        }
    }
}

/**
 * @brief 当SPI接收完成,将会调用此回调函数,可以进行协议解析或其他必须的数据处理等
 *
 * @param hspi spi handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_c::HandleTxRxCpltCallback(hspi);
}

/**
 * @brief SPI的全双工通信,和RxCpltCallback共用解析即可,这里只是形式上封装一下,不用重复写
 *        这是对HAL库的__weak函数的重写,传输使用IT或DMA模式,在传输完成时会调用此函数
 * @param hspi spi handle
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_c::HandleTxRxCpltCallback(hspi);
}
}