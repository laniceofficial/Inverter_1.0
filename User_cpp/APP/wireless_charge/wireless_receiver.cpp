#include "task_cpp.h"

#include "SEGGER_RTT.h"
#include "receiver_ask.hpp"
#include "sampling.hpp"
extern "C"
{
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
}
#define ReceiverAsk_TIMER &htim6
// --- 调试环形缓冲 ----------------------------------------------------------------
volatile DebugTraceEvent_t debug_trace_events[DEBUG_TRACE_EVENT_COUNT];
volatile uint16_t debug_trace_write_index = 0U;
volatile uint32_t debug_trace_seq = 0U;

// --- 初始化入口 ----------------------------------------------------------------
extern "C" void task_init(void)
{
    App::samplingService().init();
    App::ReceiverAsk::init();
    SEGGER_RTT_Init();
    HAL_TIM_Base_Start_IT(ReceiverAsk_TIMER);
}

// --- HAL 回调 ----------------------------------------------------------------
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    App::samplingService().handleAdcConvCpltCallback(hadc);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == ReceiverAsk_TIMER)//4khz
    {
        App::ReceiverAsk::loop();
    }
        // if (htim->Instance == TIM2)
        // {
        //     App::ReceiverAsk::loop();
        //     HAL_IncTick();
        // }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
    (void)gpioPin;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t)
{
    if (huart == &huart2)
    {
    }
}

extern "C" void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    (void)hlptim;
}

// --- 调试追踪 ----------------------------------------------------------------
extern "C" void debug_trace_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    debug_trace_write_index = 0U;
    debug_trace_seq = 0U;
}

extern "C" void debug_trace_log(uint8_t type, uint8_t value)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t index = debug_trace_write_index;
    debug_trace_events[index].seq = debug_trace_seq;
    debug_trace_seq = debug_trace_seq + 1U;
    debug_trace_events[index].cyccnt = DWT->CYCCNT;
    debug_trace_events[index].tim6_cnt = static_cast<uint16_t>(__HAL_TIM_GET_COUNTER(&htim6));
    debug_trace_events[index].type = type;
    debug_trace_events[index].value = value;

    ++index;
    if (index >= DEBUG_TRACE_EVENT_COUNT)
    {
        index = 0U;
    }
    debug_trace_write_index = index;
    __set_PRIMASK(primask);
}
