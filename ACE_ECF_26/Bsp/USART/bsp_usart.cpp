/**
 * @file bsp_usart.cpp
 * @author Lann (you@domain.com)
 * @brief
 * @version V1.0
 * @date 2025-10-18
 * @details  关于H7的缓存问题请看markdown文档，此驱动已自动处理缓存，请勿将处于DTCMRAM内的数据作为DMA缓冲区
 * @example
 *
    #define TX_BUFFER_SIZE 4
    #define RX_BUFFER_SIZE 64
    // DMA缓冲区
    uint8_t tx_buffer[TX_BUFFER_SIZE] = {'a', 'b', 'c', 'd'};
    uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
    //初始化构造串口实例
    BSP_n::USART_c* mpu;
    mpu = new BSP_n::USART_c(&huart1, RX_BUFFER_SIZE, TX_BUFFER_SIZE,
                       BSP_n::USART_c::TxType::BLOCK,
                       BSP_n::USART_c::RxType::DMA_IDLE_DOUBLE, (uint8_t*)rx_buffer,
                       nullptr, &MPUcallback);
    // 串口发送函数
    mpu->send(tx_buffer, 4, 10);
 *
 * @todo 可以将一些函数改为虚函数？例如可以将用户回调函数改为虚函数
 * @copyright Copyright (c) 2025
 *
 */
#include "bsp_usart.hpp"
#if defined(HAL_UART_MODULE_ENABLED)
using namespace BSP_n;

// 静态成员初始化
USART_c *USART_c::header = nullptr;
USART_c *USART_c::tail = nullptr;

/**
 * @brief 将USART_c实例添加到链表
 */
void USART_c::RegisterInstance(USART_c *instance)
{
  if (instance == nullptr)
    return;

  instance->next_instance = nullptr;

  if (header == nullptr)
  {
    // 链表为空
    header = instance;
    tail = instance;
  }
  else
  {
    // 链表不为空，添加到尾部
    tail->next_instance = instance;
    tail = instance;
  }
}

/**
 * @brief 从链表中移除USART_c实例
 */
void USART_c::UnregisterInstance(USART_c *instance)
{
  if (instance == nullptr || header == nullptr)
    return;

  if (header == instance)
  {
    // 要移除的是头节点
    header = header->next_instance;
    if (tail == instance)
    {
      tail = nullptr; // 如果只有一个节点
    }
    instance->next_instance = nullptr;
    return;
  }

  // 查找要移除节点的前一个节点
  USART_c *prev = header;
  while (prev != nullptr && prev->next_instance != instance)
  {
    prev = prev->next_instance;
  }

  if (prev != nullptr)
  {
    prev->next_instance = instance->next_instance;
    if (tail == instance)
    {
      tail = prev; // 更新尾指针
    }
    instance->next_instance = nullptr;
  }
}

/**
 * @brief 根据句柄查找USART_c实例
 */
USART_c *USART_c::FindInstanceByHandle(UART_HandleTypeDef *huart)
{
  USART_c *current = header;
  while (current != nullptr)
  {
    if (current->handle == huart)
    {
      return current;
    }
    current = current->next_instance;
  }
  return nullptr;
}

/***
 * @brief UART发送函数
 *
 * @param data      数据缓冲区的首地址
 * @param data_size 接受的数据长度
 * @param timeout
 * 超时时间(只在阻塞式使用)，单位是ms，如果超过设置的时间，则函数返回HAL_TIMEOUT，
 * 如果设置为HAL_MAX_DELAY，处理器就会一直等到接受到设置好的数据数量再执行下一条语句。
 * @todo 加入数据缓冲区
 */
void USART_c::send(uint8_t *data, uint16_t data_size, uint32_t timeout)
{
  switch (tx_type)
  {
  case TxType_t::DMA:
    /* code */
    HAL_UART_Transmit_DMA(handle, data, data_size);
    clean_invalidate_Dcache(data, data_size);
    break;
  case TxType_t::IT:
    HAL_UART_Transmit_IT(handle, data, data_size);
    break;
  case TxType_t::BLOCK:
    HAL_UART_Transmit(handle, data, data_size, timeout);
    break;
  default:
    break;
  }
}

/**
 * @brief 中断/DMA接收开启,用于初始化以及回调函数的重启，可直接加进构造函数中
 * @version 0.1
 * @date 2024-09-16
 **/
void USART_c::start_receive()
{
  if (rx_buffer == nullptr || rxbuf_size == 0)
  {
    while (1)
    {
    }
  }
  invalidate_Dcache(rx_buffer, rxbuf_size);
  switch (rx_type)
  {
  case RxType_t::DMA_IDLE:
    HAL_UARTEx_ReceiveToIdle_DMA(handle, rx_buffer, rxbuf_size);
    memset(rx_buffer, 0, rxbuf_size); // 清空接收缓冲区
    // 跃鹿战队的框架代码中失能了半接收中断，但经过实测并不会进入该中断，如后续测试到该中断再启用吧
    __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    break;
  case RxType_t::IT_IDLE:
    HAL_UARTEx_ReceiveToIdle_IT(handle, rx_buffer, rxbuf_size);
    memset(rx_buffer, 0, rxbuf_size); // 清空接收缓冲区
    break;
  case RxType_t::DMA_NUM:
    invalidate_Dcache(rx_buffer, rxbuf_size);
    HAL_UART_Receive_DMA(handle, rx_buffer, rxbuf_size);
    memset(rx_buffer, 0, rxbuf_size); // 清空接收缓冲区
    __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    break;
  case RxType_t::IT_NUM:
    HAL_UART_Receive_IT(handle, rx_buffer, rxbuf_size);
    memset(rx_buffer, 0, rxbuf_size); // 清空接收缓冲区
    break;
#if !defined(STM32G4)
  case RxType_t::DMA_IDLE_DOUBLE:
    // 双缓冲DMA的缓存选择在进入回调前单独处理
    initialize_dma_double_buffer();
    __HAL_DMA_DISABLE_IT(handle->hdmarx, DMA_IT_HT);
    memset(rx_buffer, 0, expected_data_length); // 清空接收缓冲区,分开清
    memset(sec_rx_buffer, 0, expected_data_length);
    break;
#endif
  default:
    while (1)
    {
      // 请重新确认接收模式
    }
    break;
  }
}

/**
 * @brief uart中断所有接收,用于重启
 * @author your name (you@domain.com)
 * @version 0.1
 * @date 2024-09-16
 **/
void USART_c::stop_receive()
{
  HAL_UART_AbortReceive(handle);
  memset(rx_buffer, 0, rxbuf_size); // 清空接收缓冲区
}

/***
 * @brief 若收到指定大小数据，则立刻返回
 * @param timeout
 * 超时时间(只在阻塞式使用)，单位是ms，如果超过设置的时间，则函数返回HAL_TIMEOUT，
 * 如果设置为HAL_MAX_DELAY，处理器就会一直等到接受到设置好的数据数量再执行下一条语句。
 * @return 返回值rx为最后接收的数据量
 */
uint16_t USART_c::block_receive(uint32_t timeout)
{
  if (rx_type == RxType_t::BLOCK_NUM)
  {
    if (HAL_UART_Receive(handle, rx_buffer, rxbuf_size, timeout) == HAL_OK)
    {
      return rxbuf_size;
    }
  }
  else if (rx_type == RxType_t::BLOCK_IDLE)
  {
    uint16_t rx_len; // rx为最后接收的数据量
    if (HAL_UARTEx_ReceiveToIdle(handle, rx_buffer, rxbuf_size, &rx_len,
                                 timeout) == HAL_OK) //
    {
      return rx_len;
    }
  }
  else
  {
    return 0; // recv failed
  }
  return 0;
}

void USART_c::handleRxCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size) // 重载函数：一般处理接收数据中断，如空闲中断
{
  USART_c *instance = FindInstanceByHandle(huart);
  if (instance->rx_callback != nullptr)
  {
#if !defined(STM32G4)
    if (instance->rx_type == RxType_t::DMA_IDLE_DOUBLE) // 如果为双缓冲中断
    {
      instance->handle_dma_double_buffer(Size);
    }
    else // 如果不是双缓冲
#endif
    {
      instance->invalidate_Dcache(instance->rx_buffer, instance->rxbuf_size);
      instance->rx_callback(instance->rx_buffer, Size); // 进自定义回调处理数据
    }
    instance->start_receive(); // 回调完成时重启
  }
}

void USART_c::handleTxCallback(UART_HandleTypeDef *huart)
{
  USART_c *instance = FindInstanceByHandle(huart);
  if (instance->handle == huart && instance->tx_callback != nullptr)
  {
    instance->tx_callback(); // 进自定义回调处理数据
  }
}
void USART_c::handleErrorCallback(UART_HandleTypeDef *huart)
{
  USART_c *instance = FindInstanceByHandle(huart);
  if (instance->handle == huart && instance->rx_callback != nullptr)
  {
    instance->stop_receive();  // 中断所有接收
    instance->start_receive(); // 重启接收
  }
}

/**
 * @brief
 * 每次dma/idle(空闲)中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  BSP_n::USART_c::handleRxCallback(huart, Size);
}

/**
 * @brief usart接收完成中断
 * @author your name (you@domain.com)
 * @version 0.1
 * @date 2024-09-16
 *
 **/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BSP_n::USART_c::handleRxCallback(huart);
}

// void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
// {
//   while (1)
//   {
//     /* code *测试是否会进入半完成中断*/
//   }
// }

/*
 *@brief 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BSP_n::USART_c::handleTxCallback(huart);
}
/**
 *  @brief 错误回调，主要用于重启usart接收，也可在此自定义
 *
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BSP_n::USART_c::handleErrorCallback(huart);
}
#endif