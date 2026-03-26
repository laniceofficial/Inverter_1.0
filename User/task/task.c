#include "task.h"
#include "collection.h"
#include "dma.h"
#include "gpio.h"
#include "hrtim.h"
#include "lptim.h"
#include "main.h"
#include "oled.h"
#include "pid.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include "svpwm.h"
#include "tim.h"
#include <stdint.h>
#include <sys/types.h>

// #include "draw_api.h"
// PC6--F1，PC8--E1
// 开关频率f = 6.8E8 / (2*HRTIM_M_HALF_CNT)
#define HRTIM_M_CNT (54400)
#define HRTIM_M_HALF_CNT (HRTIM_M_CNT / 2)
#define HRTIM_1per4_CNT (HRTIM_M_CNT / 4)
#define HRTIM_1per8_CNT (HRTIM_M_CNT / 8) // 中心对称需要除以8
#define SIN_ALL_PIONT 100                 // 100hz
#define SQRT_2_3 0.8165f                  // sqrt(2.0/3.0)
#define PI_2_3 2.0944f                    // 2*PI/3
#define SQRT3_DIV_3 0.57735f              // 1.732f/3
asm(".global _printf_float");             // oled使用printf

float M_duty = 0.9f; // 调制比

// todo 测试公式 Uab是否准确计算
// 测试闭环svpwm效果
// 实在不行就测试采集Udc后进行闭环

static int32_t sin_table1[SIN_ALL_PIONT] = {0};
// static int32_t sin_table2[SIN_ALL_PIONT] = {0};
// static int32_t sin_table3[SIN_ALL_PIONT] = {0};

PID voltage_PID;
PID current_PID;
float L_voltage_ref = 9;    // 线电压有效值参考值
float X_voltage_ref = 26.1; // 32*sqrt(2/3) 相电压最大值
float current_ref = 0;
float voltage_fdb = 0;
float current_fdb = 0;
float add_freq = 0;
collect_data_t *collect_data = NULL;
svpwm_t svpwm_v;
void task_init() {

  //  OLED_Init();
  //  OLED_Clear();
  //  OLED_Printf(0, 0, OLED_8X16,  "FREQ:        Hz");
  //  OLED_Printf(0, 16, OLED_8X16, "A:            V");
  //  OLED_Printf(0, 32, OLED_8X16, "B:            V");
  //  OLED_Printf(0, 48, OLED_8X16, "C:            V");
  //  OLED_Update();
  sinTab_genarate();
  //  svpwm_init(&svpwm_v, 0, 50, 10000);
  pid_init(&voltage_PID, PID_DELTA, V_KP, V_KI, 0, 0.5, 0.57, 0);
  //  pid_init(&current_PID, PID_DELTA, 0.01, 0, 0, 0.5, 1.2, 0);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_F);
  //  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  //  HAL_TIM_Base_Start_IT(&htim5);
  voltage_PID.ref = L_voltage_ref; // 有效值为15V
  HAL_TIM_Base_Start_IT(&htim6);
  collect_data = collection_init();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);

  // HAL_LPTIM_TimeOut_Start_IT(&hlptim1, 32, 1000);
  // HAL_SuspendTick();
  // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // HAL_ResumeTick();
  // HAL_LPTIM_TimeOut_Start_IT(&hlptim1,100,1000);
  // SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  // HAL_SuspendTick(); // 关闭系统systick中断，防止睡眠被systick中断打断
  // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,
  //                        PWR_SLEEPENTRY_WFI); // 进入WFI睡眠模式
}

void sinTab_genarate() {
  for (uint16_t i = 0; i < SIN_ALL_PIONT; i++) {
    // 每个周期覆盖完整的2π (360°)
    float angle = 2.0f * PI * i / SIN_ALL_PIONT;
    sin_table1[i] = (int32_t)(arm_sin_f32(angle) * HRTIM_1per4_CNT);
    //  sin_table2[i] = (int32_t)(arm_sin_f32(angle - 2.0f * PI / 3.0f) *
    //  HRTIM_1per4_CNT); sin_table3[i] = (int32_t)(arm_sin_f32(angle + 2.0f *
    //  PI / 3.0f) * HRTIM_1per4_CNT);
  }
}
// static float invSqrt(float x) // 平方根倒数速算法
// {
//     float halfx = 0.5f * x;
//     float y = x;
//     long i = *(long *)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float *)&i;
//     y = y * (1.5f - (halfx * y * y));
//     return y;
// }
float Uab = 0;
float actual_uab = 0;
uint16_t freq = 0;
uint8_t flag = 0;
float fl=0;
uint8_t ask_arr[] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1,
    1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1, 1,  1, 1, 1, 1, 1, 1, 1, 1,
    1,  1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, };
void PID_Seyduty() {
  voltage_PID.ref = L_voltage_ref; // 有效值为15V
  // voltage_PID.ref =arm_sin_f32(PI * cnt_temp / (SIN_ALL_PIONT/2));
  // //对有效值进行闭环 Uab = 0.0047f * Uab + Uab + 0.0009f;
  actual_uab = (collect_data->volatage[0] + collect_data->volatage[2] +
                collect_data->volatage[1]) *
               SQRT3_DIV_3;
  pid_calculate(&voltage_PID, actual_uab);
  // current_ref = voltage_PID.output;
  // pid_calculate(&current_PID, current_fdb);
  // 解算
}
// static int32_t d1 = 0;
// static int32_t d2 = 0;
// static int32_t d3 =0;

void duty_update() {

  // svpwm_v.Uref = voltage_PID.output;
  // svpwm_v.Uref = 0.5; //开环
  // svpwm_calculate(&svpwm_v);
  // 使用局部变量减少内存访问
  // float duty_a = svpwm_v.duty_a;
  // float duty_b = svpwm_v.duty_b;
  // float duty_c = svpwm_v.duty_c;
  static float cnt_temp = 0;
  uint8_t allow_ = 0;

  // int32_t duty = (sin_table1[cnt_temp] * M_duty);
  // int32_t duty2 = (sin_table2[cnt_temp] * M_duty);
  // int32_t duty3 = (sin_table3[cnt_temp] * M_duty);
  // if (cnt_temp >= SIN_ALL_PIONT) {
    // cnt_temp = 0;
  // }
  freq++;
  if (freq > 10000) {
    freq=0;
    }

    if (freq % 1000 == 0) {
      allow_ = 1;
    } else {
      allow_ = 0;
    }
  // if()
  if (flag && allow_) {
    // cnt_temp = 0;
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    cnt_temp+=0.01;
    if (cnt_temp >= 1.4) {
      cnt_temp = 0;
    }
    fl= ask_arr[(uint8_t)cnt_temp*100];
    if (ask_arr[(uint8_t)cnt_temp*100]) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
  }

  // 单相逆变
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
                         HRTIM_COMPAREUNIT_1,
                         HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
                         HRTIM_COMPAREUNIT_3,
                         HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
                         HRTIM_COMPAREUNIT_1,
                         HRTIM_M_HALF_CNT - HRTIM_1per4_CNT);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
                         HRTIM_COMPAREUNIT_3,
                         HRTIM_M_HALF_CNT + HRTIM_1per4_CNT);
  // 三相逆变
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty2);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty2);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_1per4_CNT - duty3);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_1per4_CNT + duty3); svpwm
  //  三相
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_M_HALF_CNT * duty_a);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_M_HALF_CNT * duty_a);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_M_HALF_CNT * duty_b);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_M_HALF_CNT * duty_b);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
  //  HRTIM_COMPAREUNIT_1, HRTIM_M_HALF_CNT - HRTIM_M_HALF_CNT * duty_c);
  //  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
  //  HRTIM_COMPAREUNIT_3, HRTIM_M_HALF_CNT + HRTIM_M_HALF_CNT * duty_c); aaa++;
}
uint8_t allow_PWD = 1;
void task_loop() {
  static uint8_t is_PWD = 1;
  static uint16_t temp_cnt = 0;
  if (is_PWD && allow_PWD) {
    temp_cnt = 0;
    is_PWD = 0;
    pid_reset(&voltage_PID);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                  HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                  HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
    // HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 |
    // HRTIM_OUTPUT_TB2);
  } else if (is_PWD || !allow_PWD) {
    temp_cnt++;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
    // HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 |
    // HRTIM_OUTPUT_TB2);
  }
  // UpdateScreen();
  // collection_update();
  PID_Seyduty();
  duty_update();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) // tim6负责10khz计算
  {
    task_loop(); // 任务循环
  } else if (htim == &htim5) {
    // 常低按下高
    static uint8_t detect_flag = 0;
    if (detect_flag == 0) //
    {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET) {
        // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); // 电平反转
        detect_flag = 1;
      } else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) ==
                 GPIO_PIN_SET) // 读取电平是否发生变化)
      {
        detect_flag = 2;
      } else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) ==
                 GPIO_PIN_SET) // 读取电平是否发生变化)
      {
        detect_flag = 3;
      }
    } else {
      if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET) &&
          detect_flag == 1) // 读取电平是否发生变化
      {
        detect_flag = 0;
        change_freq(&svpwm_v, svpwm_v.target_freq + 1);
      } else if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET) &&
                 detect_flag == 2) {
        detect_flag = 0;
        change_freq(&svpwm_v, svpwm_v.target_freq - 1);
      } else if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET) &&
                 detect_flag == 3) {
        detect_flag = 0;
        // change_freq(&svpwm_v, 50);
      }
    }
  }
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim) {

  HAL_ResumeTick();
  HAL_LPTIM_TimeOut_Stop_IT(hlptim);
  task_loop();
  if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET)) {
    return;
  }
  HAL_LPTIM_TimeOut_Start_IT(hlptim, 32, 1000);
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  // HAL_LPTIM_MspDeInit(&hlptim1); // 关闭LP定时器
  // SystemClock_Config();          // 配置系统时钟
  // SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
  //                 SysTick_CTRL_ENABLE_Msk; // 打开Systick的中断
  // SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;    //
  // 退出中断时不再自动进入低功耗模式
}
