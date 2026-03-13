extern "C" {
#include "main.h"
}
#if defined(HAL_UART_MODULE_ENABLED)
#    ifndef __BSP_USART_HPP
#        define __BSP_USART_HPP

#        include "usart.h"
#        include <cstdint>
#        include <cstring>
#        include <functional>
extern "C" {
}

namespace BSP_n
{
// 自定义接收回调指针
using TxCallback = std::function<void()>;
// 自定义接收回调指针，参数1:接收数组，参数2：数组长度
using RxCallback = std::function<void(const uint8_t* data, uint16_t size)>;

/* USART状态枚举 */

class USART_c
{
private:
    // 添加链表管理函数
    static void RegisterInstance(USART_c* instance);
    static void UnregisterInstance(USART_c* instance);
    static USART_c* FindInstanceByHandle(UART_HandleTypeDef* huart);

    static USART_c* header;           // 链表头指针
    static USART_c* tail;             // 链表尾指针，用于快速插入
    USART_c* next_instance = nullptr; // 指向下一个实例

public:
    /* 发送模式枚举 */
    enum class TxType_t {
        DMA,  // DMA发送
        IT,   // 中断式发送
        BLOCK // 阻塞式发送
    };

    enum class RxType_t
    {
        BLOCK_NUM,  // 阻塞接收
        DMA_NUM,    // DMA接收一定字节
        IT_NUM,     // IT接收一定字节
        BLOCK_IDLE, // 阻塞+空闲
        DMA_IDLE,   // DMA+Idle
        IT_IDLE,    // IT+Idle
        DMA_IDLE_DOUBLE // DMA+空闲+双缓冲 (G4系列不支持)
    };

    // 初始化结构体
    struct Config {
        UART_HandleTypeDef* Handle;       // 对应句柄指针
        uint16_t RxBuffSize = 0;          // 最大接收缓冲数据量,默认为0
        RxType_t RxType = RxType_t::DMA_IDLE; // 接收方式,默认为DMA空闲中断
        TxType_t TxType = TxType_t::BLOCK;    // 发送方式,默认为阻断发送
        TxCallback pTxCallback = nullptr; // 发送回调,初始化都为nullptr
        RxCallback pRxCallback = nullptr; // 接收回调
        uint8_t expected_data_length;     // 数据包应有长度
        uint8_t* pRxBuffer = nullptr;     // 第一缓冲区指针，一般只使用这个
        uint8_t* pSecRxBuffer = nullptr;  // 第二地址，使用DMA双缓冲时可用
    };
    // friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
public:
    USART_c() {}
    USART_c(UART_HandleTypeDef* handle_,
          uint16_t rxbuf_size_,
          uint8_t data_length_,
          TxType_t tx_type_,
          RxType_t rx_type_,
          uint8_t* rx_buff_ptr_,
          TxCallback tx_callback_,
          RxCallback rx_callback_) :
        handle(handle_),
        expected_data_length(data_length_),
        rx_type(rx_type_),
        tx_type(tx_type_),
        tx_callback(tx_callback_),
        rx_callback(rx_callback_),
        rx_buffer(rx_buff_ptr_),
        rxbuf_size(rxbuf_size_)
    {
        init();
    }

    USART_c(Config& config_)
    {
#if defined(STM32G4)
        // 编译时检查，确保G4不使用双缓冲模式
        if (config_.RxType == RxType_t::DMA_IDLE_DOUBLE)
        {
            static_assert(1, "STM32G4 does not support DMA double buffer mode");
        }
        
#endif
            handle = config_.Handle;
            rxbuf_size = config_.RxBuffSize;
            expected_data_length = config_.expected_data_length;
            rx_type = config_.RxType;
            tx_type = config_.TxType;
            rx_callback = config_.pRxCallback;
            tx_callback = config_.pTxCallback;
            rx_buffer = config_.pRxBuffer;
            sec_rx_buffer = config_.pSecRxBuffer;

            init();
        }
        ~USART_c() { UnregisterInstance(this); }

        void send(uint8_t *data, uint16_t data_size, uint32_t timeout = 0); // 发送函数
        void start_receive();                                               // 中断/DMA接收开启
        uint16_t block_receive(uint32_t timeout);                           // 阻断式接收
        void stop_receive();

        // 回调处理
        static void handleRxCallback(UART_HandleTypeDef *huart,
                                     uint16_t Size = 0);            // 重载函数，将hal库的回调函数引出
        static void handleTxCallback(UART_HandleTypeDef *huart);    // 将hal库的回调函数引出
        static void handleErrorCallback(UART_HandleTypeDef *huart); // 将hal库的回调函数引出
    private:
        // 初始化实例
        void init()
        {
            if (rx_type == RxType_t::DMA_IDLE_DOUBLE)
                sec_rx_buffer = rx_buffer + get_single_buffer_size();
            RegisterInstance(this);
            start_receive();
        }
        // 和构造函数相同，只是将外部接口调用出来
        void config_init(Config &config_)
        {
            rxbuf_size = config_.RxBuffSize;
            rx_type = config_.RxType;
            tx_type = config_.TxType;
            rx_callback = config_.pRxCallback;
            tx_callback = config_.pTxCallback;
            handle = config_.Handle;
            expected_data_length = config_.expected_data_length;
            rx_buffer = config_.pRxBuffer;
            sec_rx_buffer = config_.pSecRxBuffer;

            UnregisterInstance(this);
            RegisterInstance(this);
            start_receive();
        }
#if !defined(STM32G4)
    // dma双缓冲初始化
    void initialize_dma_double_buffer()
    {

        handle->ReceptionType = HAL_UART_RECEPTION_TOIDLE; // 改变接收类型为持续接收至完成或空闲

        handle->RxEventType = HAL_UART_RXEVENT_IDLE; // 接收事件为空闲中断

        handle->RxXferSize = rxbuf_size; // 设置接收长度

        SET_BIT(handle->Instance->CR3, USART_CR3_DMAR); // 将对应串口的DMA打开

        __HAL_UART_ENABLE_IT(handle, UART_IT_IDLE); // 使能空闲中断
        // 设置双缓冲数组，开始接收 :这里H7和F4不一样，Insatnce为RDR寄存器
        HAL_DMAEx_MultiBufferStart(handle->hdmarx, get_data_register_address(), (uint32_t)rx_buffer,
                                   (uint32_t)sec_rx_buffer, rxbuf_size);
        
    }
    // 处理双缓冲数据
    void handle_dma_double_buffer(uint16_t Size)
    {
        // 该标志位就是表示当接收数据的缓冲区是哪一个，如果等于0就是第一个缓冲区
        // 等于1就是第二个缓冲区
        
        if (((((DMA_Stream_TypeDef*)handle->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(handle->hdmarx); // disable DMA

            // 将当前目标缓冲区从0改到1，即开启DMA后接收数据的缓冲区为第二个
            ((DMA_Stream_TypeDef*)handle->hdmarx->Instance)->CR |= DMA_SxCR_CT;

            __HAL_DMA_SET_COUNTER(handle->hdmarx, rxbuf_size); // 设置数据长度

            // if (Size == instance->usart_data_length)
            // {
            invalidate_Dcache(rx_buffer, get_single_buffer_size());
            rx_callback(rx_buffer, Size); // 进自定义回调处理数据
            // }
        } else {
            __HAL_DMA_DISABLE(handle->hdmarx); // 失效DMA   disable DMA
            // 将当前目标缓冲区从1改到0，即开启DMA后接收数据的缓冲区为第一个
            ((DMA_Stream_TypeDef*)handle->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);

            __HAL_DMA_SET_COUNTER(handle->hdmarx, rxbuf_size);

            // if (Size == instance->usart_data_length)
            // {
            invalidate_Dcache(sec_rx_buffer, get_single_buffer_size());
            rx_callback(sec_rx_buffer, Size); // 进自定义回调处理数据
            // }
        }
        __HAL_DMA_ENABLE(handle->hdmarx);
    }

#endif
    uint32_t get_data_register_address()
    {
#        if defined(STM32H7)
        return (uint32_t)&handle->Instance->RDR;
#        elif defined(STM32F4)
        return (uint32_t)&handle->Instance->DR;
#        elif defined(STM32G4)
        return (uint32_t)&handle->Instance->RDR;
#        endif
{

}
    }

    // 获取接收模式状态
    bool is_double_buffer_mode() const { return rx_type == RxType_t::DMA_IDLE_DOUBLE; }
    // 获取单缓冲长度
    uint16_t get_single_buffer_size() const
    {
        static uint16_t single_buffer_size = 0;
        return single_buffer_size = is_double_buffer_mode() ? rxbuf_size / 2 : rxbuf_size;
    }

    // 缓存管理
    void invalidate_Dcache(uint8_t* buffer, uint16_t size) const
    {
#        if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, size);
#        endif
    }
    void clean_invalidate_Dcache(uint8_t* buffer, uint16_t size) const
    {
#        if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffer, size);
#        endif
    }
    void clean_Dcache(uint8_t* buffer, uint16_t size) const
    {
#        if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
        SCB_CleanDCache_by_Addr((uint32_t*)buffer, size);
#        endif
    }

private:
    UART_HandleTypeDef* handle;       // 对应句柄指针
    uint8_t expected_data_length;     // 数据包应有长度
    RxType_t rx_type;                   // 接收方式
    TxType_t tx_type;                   // 发送方式,
    TxCallback tx_callback;           // 发送回调
    RxCallback rx_callback;           // 接收回调
    uint8_t* rx_buffer = nullptr;     // 第一缓冲区指针，一般只使用这个
    uint8_t* sec_rx_buffer = nullptr; // 第二地址，使用DMA双缓冲时可用
    uint16_t rxbuf_size;              // 最大接收缓冲数据量
};

} // namespace BSP

#    endif /* __BSP_USART_HPP */
#endif     /* HAL_UART_MODULE_ENABLED */
