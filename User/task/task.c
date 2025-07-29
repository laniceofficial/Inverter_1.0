#include "pid.h"
#include "main.h"
#include "collection.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "task.h"
#include "hrtim.h"
#include "arm_math.h"
#define user_abs(x) ((x) > (0) ? (x) : (-(x)))
#define TIM_CNT 8500
#define TIM_HALF_CNT 4250
#define HRTIM_M_HALF_CNT 17000
#define HRTIM_1per4_CNT 8500
#define SIN_ALL_PIONT 400
#define M_duty 0.85f //调制比

static int16_t sin_table[SIN_ALL_PIONT] = {0};

PID voltage_PID;
PID current_PID;
 float voltage_ref = 0;
 float current_ref = 0;
 float voltage_fdb = 0;
 float current_fdb = 0;
collect_data_t * collect_data = NULL;
void task_init()
{

    sinTab_genarate();
    collect_data = collection_init();
    pid_init(&voltage_PID, PID_POSITION, 0.01, 0, 0, 0.5, 2, 0);
    pid_init(&current_PID, PID_POSITION, 0.01, 0, 0, 0.5, 1.2, 0);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1|HRTIM_OUTPUT_TF2);
    HAL_TIM_Base_Start_IT(&htim6);
    voltage_PID.ref = 15; //有效值为15V
}
inline void sinTab_genarate()
{
    for (uint16_t i = 0; i < SIN_ALL_PIONT; i++)
    {
        uint16_t half_point = SIN_ALL_PIONT / 2;
        sin_table[i] = (int16_t)(arm_sin_f32(PI * i / half_point) * HRTIM_1per4_CNT);
    }
}

inline void PID_Seyduty()
{
    // voltage_PID.ref =arm_sin_f32(PI * cnt_temp / (SIN_ALL_PIONT/2)); //对有效值进行闭环
    pid_calculate(&voltage_PID, voltage_fdb);
    current_ref = voltage_PID.output;

    pid_calculate(&current_PID, current_fdb);
}   
inline void duty_update()
{
    float modulation_index = current_PID.output;
    static uint16_t cnt_temp = 0;
    int16_t duty = (sin_table[cnt_temp] * M_duty);
    cnt_temp++;
    if (cnt_temp >= SIN_ALL_PIONT)
    {
        cnt_temp = 0;
    }

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT + duty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_4, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT - duty);
}
inline void task_loop()
{
    collection_update();
    voltage_fdb = collect_data->volatage;
    current_fdb = collect_data->current;
    PID_Seyduty();
    duty_update();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) // tim6负责20khz计算
    {
        task_loop(); // 任务循环
    }
}