#include "pid.h"
#include "main.h"
#include "collection.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "task.h"
#include "hrtim.h"
#include "svpwm.h"
// #include "draw_api.h"
#define TIM_CNT 8500
#define TIM_HALF_CNT 4250

#define HRTIM_M_HALF_CNT 17000
#define HRTIM_1per4_CNT 8500
#define SIN_ALL_PIONT 400
float M_duty = 0.8f; // 调制比

// todo 测试公式 Uab是否准确计算
//测试闭环svpwm效果
//实在不行就测试采集Udc后进行闭环

static int32_t sin_table1[SIN_ALL_PIONT] = {0};
static int32_t sin_table2[SIN_ALL_PIONT] = {0};
static int32_t sin_table3[SIN_ALL_PIONT] = {0};

PID voltage_PID;
PID current_PID;
float L_voltage_ref = 32; //线电压有效值参考值
float X_voltage_ref = 26.1; // 32*sqrt(2/3)
float current_ref = 0;
float voltage_fdb = 0;
float current_fdb = 0;
collect_data_t *collect_data = NULL;
svpwm_t svpwm_v;
 void task_init()
 {
    //  InitGraph();
    //  SetScreenBuffer(); // 设置OLED屏幕缓存地址
    //  SetFontSize(2);
     //  OLED_ShowCHinese(0, 00,"雾");
     //  OLED_ShowCHinese(0, 20,"雨");
    //   OLED_ShowCHinese(0, 40,"亮");
    //  OLED_CLS();
    //  SetFillcolor(pix_white);      // 白色为绘制 黑色为擦除
    //  DrawRect1(0, 16, 0 + 16, 63); // 柱状图
    //  DrawRect1(20, 16, 20 + 16, 63);
    //  DrawRect1(40, 16, 40 + 16, 63);

     sinTab_genarate();
     svpwm_init(&svpwm_v, 0, 50, 20000);
     collect_data = collection_init();
     pid_init(&voltage_PID, PID_POSITION, 0.01, 0, 0, 0.5, 2, 0);
     pid_init(&current_PID, PID_POSITION, 0.01, 0, 0, 0.5, 1.2, 0);
     HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
     HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
     HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);
     HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
     HAL_TIM_Base_Start_IT(&htim6);
     voltage_PID.ref = X_voltage_ref; // 有效值为15V
 }

 inline void sinTab_genarate()
 {
     for (uint16_t i = 0; i < SIN_ALL_PIONT; i++)
     {
         // 每个周期覆盖完整的2π (360°)
         float angle = 2.0f * PI * i / SIN_ALL_PIONT;
         sin_table1[i] = (int32_t)(arm_sin_f32(angle) * HRTIM_1per4_CNT);
         sin_table2[i] = (int32_t)(arm_sin_f32(angle - 2.0f * PI / 3.0f) * HRTIM_1per4_CNT);
         sin_table3[i] = (int32_t)(arm_sin_f32(angle + 2.0f * PI / 3.0f) * HRTIM_1per4_CNT);
     }
}
float invSqrt(float x) // 平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
inline void PID_Seyduty()
{
    // float U_alpha = invSqrt(3/2)*(collect_data->volatage[0]-collect_data->volatage[1] /2-collect_data->volatage[2]/2);
    // float U_beta = invSqrt(3 / 2) * (invSqrt(4 / 3) * collect_data->volatage[1] - invSqrt(4 / 3) *collect_data->volatage[2]);
    // voltage_PID.ref =arm_sin_f32(PI * cnt_temp / (SIN_ALL_PIONT/2)); //对有效值进行闭环
    float temp = (collect_data->volatage[0] + collect_data->volatage[2] + collect_data->volatage[1]);
    //平方
    temp *= temp;
    temp+= collect_data->volatage[0]*collect_data->volatage[1]+collect_data->volatage[0]*collect_data->volatage[2]+collect_data->volatage[1]*collect_data->volatage[2];
    temp *= 0.444444f;
    float Uab = 0;
    arm_sqrt_f32(temp, &Uab);

    pid_calculate(&voltage_PID, voltage_fdb);
    // current_ref = voltage_PID.output;
    // pid_calculate(&current_PID, current_fdb);
    //解算
    // svpwm_v.Uref = voltage_PID.output;
    svpwm_v.Uref = 0.5; //开环
    svpwm_calculate(&svpwm_v);
}
// static int32_t d1 = 0;
// static int32_t d2 = 0;
// static int32_t d3 =0;
uint64_t aaa= 0;
inline void duty_update()
{
    // float modulation_index = current_PID.output;
    static uint16_t cnt_temp = 0;
    int32_t duty = (sin_table1[cnt_temp] * M_duty);
    int32_t duty2 = (sin_table2[cnt_temp] * M_duty);
    int32_t duty3 = (sin_table3[cnt_temp] * M_duty);
    cnt_temp++;
    if (cnt_temp >= SIN_ALL_PIONT)
    {
        cnt_temp = 0;
    }
    //单相逆变
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT + duty);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT - duty);
    //三相逆变
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty2);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty3);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty3);

    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT * svpwm_v.duty_a);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT * svpwm_v.duty_a);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT * svpwm_v.duty_b);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT * svpwm_v.duty_b);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT * svpwm_v.duty_c);
    // __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT * svpwm_v.duty_c);
    aaa++;
}
inline void task_loop()
{
    static uint8_t is_PWD = 1;
    static uint16_t temp_cnt = 0;
    if(temp_cnt>2000 && is_PWD)
    {
        temp_cnt = 0;
        is_PWD = 0;
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    }
    else if(is_PWD)
    {
        temp_cnt++;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    }
    // UpdateScreen();
    collection_update();
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