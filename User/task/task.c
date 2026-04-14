#include "task.h"

#include <stdint.h>
#include <sys/types.h>

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

#define HRTIM_M_CNT (54400)
#define HRTIM_M_HALF_CNT (HRTIM_M_CNT / 2)
#define HRTIM_1per4_CNT (HRTIM_M_CNT / 4)
#define HRTIM_1per8_CNT (HRTIM_M_CNT / 8)
#define SIN_ALL_PIONT 100
#define SQRT_2_3 0.8165f
#define PI_2_3 2.0944f
#define SQRT3_DIV_3 0.57735f
#define LOW_POWER_LPTIM_TICKS 500U
#define ASK_PROBE_WINDOW_TICKS 200U
#define CHARGE_BUTTON_GPIO_Port GPIOC
#define CHARGE_BUTTON_Pin GPIO_PIN_10

extern void SystemClock_Config(void);

static void SlowEnable(void);
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

uint16_t freq = 0U;
uint8_t flag = 0U;
uint8_t arr_value = 0U;
uint8_t allow_PWD = 1U;
uint32_t cnt = 0U;
uint8_t ask_arr[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

void task_init(void)
{
    collection_init();
    g_collection_running = 1U;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    enter_low_power_mode();
}

void task_loop(void)
{
    switch (now_state)
    {
        case LOW_POWER:
            enter_low_power_mode();
            return;

        case PreDetect:
            start_low_power_probe();
            return;

        case ASKDetect:
            cnt++;
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
            enter_low_power_mode();
            return;
    }
}

void duty_update(void)
{
    static float cnt_temp = 0.0f;
    uint8_t allow_ = 0U;

    freq++;
    if (freq > 10000U)
    {
        freq = 0U;
    }

    if ((freq % 100U) == 0U)
    {
        allow_ = 1U;
    }

    if ((flag != 0U) && (allow_ != 0U))
    {
        cnt_temp += 0.01f;
        if (cnt_temp >= 0.2f)
        {
            cnt_temp = 0.0f;
        }

        arr_value = ask_arr[(uint8_t)(cnt_temp * 100.0f)];
        if (ask_arr[(uint8_t)(cnt_temp * 100.0f)] != 0U)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        }
    }

    set_normal_waveform();
}

void task_try_enter_low_power(void)
{
    if (now_state != LOW_POWER)
    {
        return;
    }

    if (g_lptim_running == 0U)
    {
        if (HAL_LPTIM_TimeOut_Start_IT(&hlptim1, LOW_POWER_LPTIM_TICKS, LOW_POWER_LPTIM_TICKS) == HAL_OK)
        {
            g_lptim_running = 1U;
        }
    }

    if (g_lptim_running == 0U)
    {
        return;
    }

    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
    HAL_ResumeTick();

    if (now_state == PreDetect)
    {
        ensure_tim6_started();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        task_loop();
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
    if (GPIO_Pin != CHARGE_BUTTON_Pin)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(CHARGE_BUTTON_GPIO_Port, CHARGE_BUTTON_Pin) != GPIO_PIN_SET)
    {
        return;
    }

    if (g_lptim_running != 0U)
    {
        HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
        g_lptim_running = 0U;
    }

    now_state = PreDetect;
}

ChangeState_e GetNowState(void)
{
    return now_state;
}

void SetNowState(ChangeState_e state)
{
    now_state = state;
}

static void SlowEnable(void)
{
    static uint8_t is_PWD = 1U;
    static uint16_t temp_cnt = 0U;

    if ((is_PWD != 0U) && (allow_PWD != 0U))
    {
        temp_cnt = 0U;
        is_PWD = 0U;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
    }
    else if ((is_PWD != 0U) || (allow_PWD == 0U))
    {
        temp_cnt++;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
    }
}

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

    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
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
}

static void start_low_power_probe(void)
{
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
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1,
                           HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3,
                           HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1,
                           HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3,
                           HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
}
