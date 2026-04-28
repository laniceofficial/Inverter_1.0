#include "task.h"

#include <stdint.h>
#include <sys/types.h>

#include "SEGGER_RTT.h"
#include "arm_math.h"
#include "collection.h"
#include "dma.h"
#include "gpio.h"
#include "hrtim.h"
#include "lptim.h"
#include "main.h"
#include "oled.h"
#include "pid.h"
#include "stm32_hal_legacy.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "tim.h"
#include "usart.h"
#define HRTIM_M_CNT (38857)
#define HRTIM_M_HALF_CNT (HRTIM_M_CNT / 2)
#define HRTIM_1per4_CNT (HRTIM_M_CNT / 4)
#define HRTIM_1per8_CNT (HRTIM_M_CNT / 8)
#define SIN_ALL_PIONT 100
#define SQRT_2_3 0.8165f
#define PI_2_3 2.0944f
#define SQRT3_DIV_3 0.57735f
#define LOW_POWER_LPTIM_TICKS 500U
#define ASK_PROBE_WINDOW_TICKS 100U
#define CHARGE_BUTTON_GPIO_Port GPIOC
#define CHARGE_BUTTON_Pin GPIO_PIN_15
// asm(".global _printf_float");
extern void SystemClock_Config(void);

// static void SlowEnable(void);
static uint8_t is_charge_button_pressed(void);
static void ensure_tim6_started(void);
static void ensure_tim6_stopped(void);
static void enter_low_power_mode(void);
static void start_low_power_probe(void);
static void resume_active_mode(void);
static void set_normal_waveform(void);
static void power_stage_enable_low_power_probe(void);
static void power_stage_enable_charging(void);
static void power_stage_disable(void);

float M_duty = 0.9f;
float add_freq = 0.0f;

static ChangeState_e now_state = LOW_POWER;
static uint8_t g_tim6_running = 0U;
static uint8_t g_lptim_running = 0U;
static uint8_t g_collection_running = 0U;
static uint16_t g_probe_ticks = 0U;
static volatile uint16_t g_signal_divider_tick = 0U;
static volatile uint8_t g_signal_pattern_index = 0U;

volatile uint16_t freq = 0U;
volatile uint8_t flag = 1U;
volatile uint8_t arr_value = 0U;
volatile DebugTraceEvent_t debug_trace_events[DEBUG_TRACE_EVENT_COUNT];
volatile uint16_t debug_trace_write_index = 0U;
volatile uint32_t debug_trace_seq = 0U;
uint8_t allow_PWD = 1U;
uint32_t cnt = 0U;
uint8_t uart_buf[20] = {0};
uint8_t ask_arr[] = {
    1,
    0,
};
uint8_t valid_ask_arr[] = {
   0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    // 1,0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 
};
void task_init(void)
{
    collection_init();
    SEGGER_RTT_Init();
    // debug_trace_init();
    g_collection_running = 1U;
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_buf, 10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    power_stage_enable_charging();
    ensure_tim6_started();
    // enter_low_power_mode();
}

void debug_trace_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    debug_trace_write_index = 0U;
    debug_trace_seq = 0U;
}

void debug_trace_log(uint8_t type, uint8_t value)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t index;

    __disable_irq();
    index = debug_trace_write_index;
    debug_trace_events[index].seq = debug_trace_seq++;
    debug_trace_events[index].cyccnt = DWT->CYCCNT;
    debug_trace_events[index].tim6_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
    debug_trace_events[index].type = type;
    debug_trace_events[index].value = value;

    index++;
    if (index >= DEBUG_TRACE_EVENT_COUNT)
    {
        index = 0U;
    }
    debug_trace_write_index = index;
    __set_PRIMASK(primask);
}

void task_loop(void)
{
    switch (now_state)
    {
        case LOW_POWER:
            enter_low_power_mode();
            return;

        case PreDetect:
            SEGGER_RTT_TerminalOut(0, "ASK_DETECT\r\n");
            start_low_power_probe();
            return;

        case ASKDetect:
            if ((get_ask_valid() != 0U) && (is_charge_button_pressed() != 0U))
            {
                resume_active_mode();
                return;
            }

            if (g_probe_ticks > 0U)
            {
                g_probe_ticks--;
            }

            if (g_probe_ticks == 0U)
            {
                enter_low_power_mode();
            }
            return;

        case PreChange:
            SEGGER_RTT_TerminalOut(0, "START_CHARGE\r\n");
            if ((get_ask_valid() == 0U) || (is_charge_button_pressed() == 0U))
            {
                enter_low_power_mode();
                return;
            }

            now_state = Changing;
            duty_update();
            return;

        case Changing:
            if ((get_ask_valid() == 0U) || (is_charge_button_pressed() == 0U))
            {
                enter_low_power_mode();
                return;
            }

            duty_update();
            return;

        default:
            SEGGER_RTT_TerminalOut(0, "ERROR\r\n");
            enter_low_power_mode();
            return;
    }
}
volatile uint8_t allow_ = 0U;
volatile uint16_t signal_freq = 1; // 默认1000hz
// 10hz--100，50hz--20，100hz--10，200hz--5，500hz--4,2000hz--1
void duty_update(void)//4000hz
{
    uint8_t next_value = 0U;

    freq++;
    if (signal_freq == 0U)
    {
        signal_freq = 1U;
    }

    g_signal_divider_tick++;
    if (g_signal_divider_tick < signal_freq)
    {
        allow_ = 0U;
        set_normal_waveform();
        return;
    }

    g_signal_divider_tick = 0U;
    allow_ = 1U;

    if ((flag != 0U))
    {
        g_signal_pattern_index++;
        if (g_signal_pattern_index >= (uint8_t)(sizeof(valid_ask_arr) / sizeof(valid_ask_arr[0])))
        {
            g_signal_pattern_index = 0U;
        }

        next_value = valid_ask_arr[g_signal_pattern_index];
    }
    else
    {
        g_signal_pattern_index++;
        if (g_signal_pattern_index >= (uint8_t)(sizeof(valid_ask_arr) / sizeof(valid_ask_arr[0])))
        {
            g_signal_pattern_index = 0U;
        }

        next_value = valid_ask_arr[g_signal_pattern_index];
    }

    arr_value = next_value;
    // debug_trace_log(DEBUG_TRACE_TYPE_ARR, next_value);
    if (next_value != 0U)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }

    allow_ = 0U;
    set_normal_waveform();
}

void task_try_enter_low_power(void)
{
    if (now_state != LOW_POWER)
    {
        return;
    }
    if (g_lptim_running == 1 && now_state != LOW_POWER)
    {
        return;
    }
    if (g_lptim_running == 0U) // 进低功耗
    {
        // if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, LOW_POWER_LPTIM_TICKS, LOW_POWER_LPTIM_TICKS) == HAL_OK)
        // {
        g_lptim_running = 1U;

        // }
    }


    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // PWR_STOPENTRY_WFE
    SystemClock_Config();
    HAL_ResumeTick();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        // task_loop();
        duty_update();
    }
    //   if (htim->Instance == TIM20) {
    //     HAL_IncTick();
    //   }
      if (htim->Instance == TIM2) {
        HAL_IncTick();
      }
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim != &hlptim1)
    {
        return;
    }

    HAL_LPTIM_TimeOut_Stop_IT(hlptim);
    g_lptim_running = 0U;
    now_state = PreDetect;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (HAL_GPIO_ReadPin(CHARGE_BUTTON_GPIO_Port, CHARGE_BUTTON_Pin) == GPIO_PIN_SET)
    {
        if (g_lptim_running != 0U)
        {
            resume_active_mode();
            collection_start();
            // HAL_LPTIM_TimeOut_Stop_IT(&hlptim1); //stopIt
            g_lptim_running = 0U; // tuichu
        }

        now_state = PreChange;
        return;
    }
    else if (HAL_GPIO_ReadPin(FCHAN_GPIO_Port, FCHAN_Pin) == GPIO_PIN_SET)
    {
        flag = 0; // 10hz现在先当作变频用
        g_signal_divider_tick = 0U;
        g_signal_pattern_index = 0U;
    }
    else if (HAL_GPIO_ReadPin(F100HZ_GPIO_Port, F100HZ_Pin) == GPIO_PIN_SET)
    {
        flag = 1;
        signal_freq = 10;
        g_signal_divider_tick = 0U;
        g_signal_pattern_index = 0U;
    }
    else if (HAL_GPIO_ReadPin(F1KHZ_GPIO_Port, F1KHZ_Pin) == GPIO_PIN_SET)
    {
        flag = 1;
        signal_freq = 1;
        g_signal_divider_tick = 0U;
        g_signal_pattern_index = 0U;
    }

}

ChangeState_e GetNowState(void)
{
    return now_state;
}

void SetNowState(ChangeState_e state)
{
    now_state = state;
}

// static void SlowEnable(void)
// {
//     static uint8_t is_PWD = 1U;
//     static uint16_t temp_cnt = 0U;

//     if ((is_PWD != 0U) && (allow_PWD != 0U))
//     {
//         temp_cnt = 0U;
//         is_PWD = 0U;
//         HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
//         HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
//     }
//     else if ((is_PWD != 0U) || (allow_PWD == 0U))
//     {
//         temp_cnt++;
//         HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
//         HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
//     }
// }

static uint8_t is_charge_button_pressed(void)
{
    return (uint8_t)(HAL_GPIO_ReadPin(CHARGE_BUTTON_GPIO_Port, CHARGE_BUTTON_Pin) == GPIO_PIN_SET);
}

static void ensure_tim6_started(void)
{
    if (g_tim6_running == 0U)
    {
        HAL_TIM_Base_Start_IT(&htim6);
        g_tim6_running = 1U;
    }
}

static void ensure_tim6_stopped(void)
{
    if (g_tim6_running != 0U)
    {
        HAL_TIM_Base_Stop_IT(&htim6);
        g_tim6_running = 0U;
    }
}

static void power_stage_enable_low_power_probe(void)
{
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, 13600);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, 40800);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, 13600);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, 40800);
}

static void power_stage_enable_charging(void)
{
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);
    // HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
    // HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    set_normal_waveform();
}

static void power_stage_disable(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);

    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_F);
    HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_MASTER);
}

static void enter_low_power_mode(void)
{
    power_stage_disable();

    if (g_collection_running != 0U)
    {
        collection_stop();
        g_collection_running = 0U;
    }

    collection_reset_ask_valid();
    ensure_tim6_stopped();

    g_probe_ticks = 0U;
    now_state = LOW_POWER;
    task_try_enter_low_power();
}

static void start_low_power_probe(void)
{
    // SEGGER_RTT_TerminalOut(0, "LOW_POWER\r\n");
    collection_reset_ask_valid();
    collection_start();
    g_collection_running = 1U;

    power_stage_enable_low_power_probe();
    ensure_tim6_started();

    g_probe_ticks = ASK_PROBE_WINDOW_TICKS;
    now_state = ASKDetect;
}

static void resume_active_mode(void)
{
    power_stage_enable_charging();
    ensure_tim6_started();

    g_probe_ticks = 0U;
    now_state = PreChange;
}

static void set_normal_waveform(void)
{
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, 2000);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, 27000);
}
// test
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart == &huart2)
    {}
}
