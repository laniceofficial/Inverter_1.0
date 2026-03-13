# readme
看readme之前可以看看这篇文章
https://blog.csdn.net/qq_38023025/article/details/104698387

## 注意/attention
- 本CPP用的电机为律为57步进电机
- 电机驱动器为 律为驱动器
- 务必知道接线方式为共阴还是共阳，在HPP的定义中改动
- cube记得使能PWM及其中断
- 传入参数时确定好驱动器设置的细分

## 接线 
如链接的文章说的，你需要在驱动器达成共阴或共阳接线
如下
```
#define CONNECTION_MOED 1u
// 共阴极接线---1；共阳极接线---0
// 共阴极：分别将Pul-，Dir-，EN-连接到控制系统的地端（与电源地隔离）；此时脉冲输入信号通过Pul+加入，Dir+；EN+在低电平有效
// 共阳极：分别将Pul+，Dir+，EN+连接到STM32板子的输出电压上，脉冲输入信号通过Pul-接入；此时，Dir-控制方向；EN-在高电平有效。即当EN-引脚低电平时，电机失能
// 改动 CONNECTION_MOED 这个值，会影响到设置电平的高低，从而改动电机的使失能和方向
```
## 配置与说明
 1. cube: 需要在cube使能一个定时器TIM，输出一路PWM方波，测试例子为 ARR = 1000-1; Period = 168-1
 2. cube: 使能中断(角度模式需要)，为global interrupt 或者 update interrupt
 3. 程序： 创建一个pwm类，再创建一个步进电机初始化结构体，通过创建一个步进电机类并向构造函数传入前两个参数，或者使用init函数初始化
 4. 速度模式：直接占空比为50%
 5. 角度模式：在HAL_TIM_PeriodElapsedCallback 中断回调中通过一个个计数的方式记录步进值，达到转动固定角度的方法

## 代码
该例子需要先创建一个pwm类，再创建一个步进电机初始化结构体，通过创建一个步进电机类并向构造函数传入前两个参数，或者使用init函数初始化
- 关于pwm请参照bsp_pwm.cpp
```
bsp_pwm_n::pwm_init_t pwm_init = {
    .htim = &htim1,
    .channel = TIM_CHANNEL_1,
    .first_value = 0,
    // TIM_HandleTypeDef *htim;
    // uint32_t channel;
                                  // pwm模式(可选多种)
    // pwm_overflow_callback pwm_overflow_callback_ptr; // 溢出中断（没有给null）
    // pwm_compare_callback pwm_compare_callback_ptr;   // 比较中断（没有给null）
    // uint8_t is_HobbyWing;                            // 是否用于好盈电调控制
    // uint16_t first_value = 0;                        // 初始值设置
    };
  stepin_init_t step_ = {
        .dir_pin_ = GPIO_PIN_11, // 方向为PE11
        .dir_port_ = GPIOE,
        .enable_pin_ = GPIO_PIN_13, // 使能为PE13
        .enable_port_ = GPIOE,
        .PerRound_Pulse = (uint8_t)800,
    };
    stepin_motor_c temp(step_);
   temp.StepIn_init(pwm_init, step_);
```

