#ifndef __LQR_H
#define __LQR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Include */
#ifdef STM32F405xx
#include "stm32f405xx.h"
#elif STM32H723xx
#include "stm32h723xx.h"
#endif
#include "cmsis_os.h"
#include <string.h>

#ifdef __cplusplus
}

/* Define */
#ifndef lqr_abs
#define lqr_abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

namespace alg_n
{

  typedef struct
  {
    float k0;
    float k1;
  } system_K_t; // lqr控制参数：计算出来的k增益矩阵

  typedef union
  {
    system_K_t struct_k;
    float array_k[2];
  } system_K_u;

  // lqr算法类
  class Lqr_c
  {
    typedef struct
    {
      uint8_t System_State_Size = 0; // 对应u与input
      uint8_t Control_Size = 0;      // 对应Output
      // 非线性控制量
      float Control_Variable = 0;
      float Control_Area = 0; // 控制区域

      float *Input = nullptr;
      float *Output = nullptr;
      float *k = nullptr; // 最优反馈增益矩阵

      float Ki = 0; // 等于零不使用积分
      float max_integral = 0;
      float output_limit = 0;
      float dt = 0;
      float Ierror = 0;
      void (*User_Func_f)(void);

    } lqr_alg_t;

  public:
    Lqr_c(uint8_t system_state_size, uint8_t control_size, float *k, float Ki_, float max_integral_, float output_limit_, float dt_);
    void Init(uint8_t system_state_size, uint8_t control_size, float *k, float Ki_, float max_integral_, float output_limit_, float dt_);
    void dataUpdate(float *system_state); // 传入当前状态：如[角度误差，角速度误差] 角速度目标为0
    float Calc(void);
    void Clear(void);
    inline void SetKcoef(float *K);
    inline void SetKi(float Ki);

  private:
    lqr_alg_t lqr_data_;
  };
  inline void Lqr_c::SetKcoef(float *K_)
  {
    lqr_data_.k[0] = K_[0];
    lqr_data_.k[1] = K_[1];
  }
  inline void Lqr_c::SetKi(float Ki)
  {
    lqr_data_.Ki = Ki;
  }
}

#endif

#endif
