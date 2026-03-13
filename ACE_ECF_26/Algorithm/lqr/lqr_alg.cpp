/**
 * @file lqr_alg.cpp
 * @brief lqr算法层
 *
 * @version 1.1
 * @date 2025-10
 */

#include "lqr_alg.hpp"
using namespace alg_n;

/**
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 * @brief LQR计算
 * @param  void
 * @return output
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 */
float alg_n::Lqr_c::Calc(void) {
  int i, j;
  for (i = 0; i < lqr_data_.Control_Size; i++) {
    lqr_data_.Output[i] = 0;
    for (j = 0; j < lqr_data_.System_State_Size; j++) {
      lqr_data_.Output[i] -= lqr_data_.Input[j] *
                             lqr_data_.k[i * lqr_data_.System_State_Size +
                                         j]; // 按照公式其实此处为-=，而之前为+=
    }
  }
  if (lqr_data_.Ki != 0) {
    float error = lqr_data_.Input[0];
    lqr_data_.Ierror += lqr_data_.Ki * lqr_data_.dt * error;
  }
  return lqr_data_.Output[0];
}

/**
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 * @brief  LQR构造初始化
 * @param  system_state_size
 * @param  control_size
 * @param  k
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 */
Lqr_c::Lqr_c(uint8_t system_state_size, uint8_t control_size, float *k,
             float Ki_, float max_integral_, float output_limit_, float dt_)

{
  lqr_data_.System_State_Size = system_state_size;
  lqr_data_.Control_Size = control_size;
  lqr_data_.Ki = Ki_;
  lqr_data_.max_integral = max_integral_;
  lqr_data_.output_limit = output_limit_;
  lqr_data_.dt = dt_;
  // lqr->Control_Area = control_area;
  if (system_state_size != 0) {
    lqr_data_.Input =
        (float *)user_malloc(sizeof(float) * system_state_size * control_size);
    memset(lqr_data_.Input, 0,
           sizeof(float) * system_state_size * control_size);
    lqr_data_.k =
        (float *)user_malloc(sizeof(float) * system_state_size * control_size);
    memset(lqr_data_.k, 0, sizeof(float) * system_state_size * control_size);
  }
  if (control_size != 0) {
    lqr_data_.Output = (float *)user_malloc(sizeof(float) * control_size);
    memset(lqr_data_.Output, 0, sizeof(float) * control_size);
  }

  lqr_data_.k = k;
}

/**
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 * @brief  LQR手动初始化
 * @param  system_state_size
 * @param  control_size
 * @param  k    lqr参数
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 */
void Lqr_c::Init(uint8_t system_state_size, uint8_t control_size, float *k,
                 float Ki_, float max_integral_, float output_limit_,
                 float dt_) {
  lqr_data_.System_State_Size = system_state_size;
  lqr_data_.Control_Size = control_size;
  lqr_data_.Ki = Ki_;
  lqr_data_.max_integral = max_integral_;
  lqr_data_.output_limit = output_limit_;
  lqr_data_.dt = dt_;
  // lqr->Control_Area = control_area;
  if (system_state_size != 0) {
    lqr_data_.Input =
        (float *)user_malloc(sizeof(float) * system_state_size * control_size);
    memset(lqr_data_.Input, 0,
           sizeof(float) * system_state_size * control_size);
    lqr_data_.k =
        (float *)user_malloc(sizeof(float) * system_state_size * control_size);
    memset(lqr_data_.k, 0, sizeof(float) * system_state_size * control_size);
  }
  if (control_size != 0) {
    lqr_data_.Output = (float *)user_malloc(sizeof(float) * control_size);
    memset(lqr_data_.Output, 0, sizeof(float) * control_size);
  }

  lqr_data_.k = k;
}

/**
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 * @brief  LQR数据清除
 * @param  void
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 */
void Lqr_c::Clear(void) {
  memset(lqr_data_.Input, 0,
         sizeof(float) * lqr_data_.System_State_Size * lqr_data_.Control_Size);

  memset(lqr_data_.Output, 0, sizeof(float) * lqr_data_.Control_Size);
}

/**
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 * @brief  LQR数据更新
 * @param  system_state  系统状态矩阵
 * ************************* Dongguan-University of Technology
 * -ACE**************************
 */
void Lqr_c::dataUpdate(float *system_state) {
  int i = 0;

  for (; i < lqr_data_.System_State_Size; i++) {
    lqr_data_.Input[i] = system_state[i];
  }
}
