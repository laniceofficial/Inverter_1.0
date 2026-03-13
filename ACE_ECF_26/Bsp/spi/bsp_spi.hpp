extern "C"
{
#include "main.h"
}
#if defined(HAL_SPI_MODULE_ENABLED)
#ifndef __BSP_SPI_HPP
#define __BSP_SPI_HPP

/* 根据开发板引出的spi引脚以及CubeMX中的初始化配置设定 */
#define SPI_DEVICE_CNT 2       // SPI总线数目
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机最多数目

#ifdef __cplusplus
extern "C"
{
#endif

#include "spi.h"
#include "stdint.h"
#include "stm32g431xx.h"
#include "main.h"
#ifdef __cplusplus
}
#endif
namespace BSP_n
{
    // BMI088初始化配置的前向声明
    struct SPI_InitConfig_s;


    class SPI_c
    {
    public:
        /* spi transmit recv mode enumerate*/
        enum class TransType_t
        {
            DMA,  // DMA发送
            IT,   // 中断式发送
            BLOCK // 阻塞式发送
        };

        SPI_c(SPI_InitConfig_s config);
        SPI_c() {
        }
        // 外部调用函数
        void Init(SPI_InitConfig_s config);
        void Init(SPI_HandleTypeDef *spi_handle,
                  GPIO_TypeDef *GPIOx,
                  uint16_t cs_pin,
                  TransType_t spi_work_mode = TransType_t::IT,
                  void (*spi_module_callback)(SPI_c *register_instance) = nullptr);
        void Transmit(uint8_t *ptr_data, uint8_t len);
        void Receive(uint8_t *ptr_data, uint8_t len);
        void TransRecv(uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);
        void SetMode(TransType_t spi_mode);
        static void HandleTxRxCpltCallback(SPI_HandleTypeDef *hspi);

    protected:
        uint8_t *rx_buffer_;                               // 本次接收的数据缓冲区
        static SPI_c *spi_instance_[MX_SPI_BUS_SLAVE_CNT]; // SPI实例指针数组
        static uint8_t idx_;
        static uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT];
        SPI_HandleTypeDef *spi_handle_; // SPI外设handle
        uint8_t rx_size_;               // 本次接收的数据长度
        TransType_t spi_work_mode_;     // 传输工作模式

        GPIO_TypeDef *GPIOx_; // 片选信号对应的GPIO,如GPIOA,GPIOB等等
        uint16_t cs_pin_;     // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
        uint8_t CS_State_;    // 片选信号状态,用于中断模式下的片选控制
        // 接收的回调函数,用于解析接收到的数据
        void (*spi_module_callback)(SPI_c *register_instance); // 接收回调函数
    };

    /* SPI初始化配置 */
    struct SPI_InitConfig_s
    {
        SPI_HandleTypeDef *spi_handle; // SPI外设handle
        // SPI的CSS引脚的GPIO设置
        GPIO_TypeDef *GPIOx;                                   // 片选信号对应的GPIO,如GPIOA,GPIOB等等
        uint16_t cs_pin;                                       // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
        SPI_c::TransType_t spi_work_mode;                      // 传输工作模式
        void (*spi_module_callback)(SPI_c *register_instance); // 接收回调函数
    };
} // namespace BSP
#endif /* BSP_SPI_HPP */
#endif /* HAL_UART_MODULE_ENABLED */