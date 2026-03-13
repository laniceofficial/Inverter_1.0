/**
 * @file Alg_PID.cpp
 * @brief PID控制算法
 *
 * @version 2.1
 * @date 2024-9-16  1.0 加入了梯形积分
 *       2024-10-3  1.1 实际使用发现还是使用初始化函数方便一点
 *       2024-10-22 2.0 并未发现野指针的问题（我觉得没有），修正了变积分和微分先行算法
 *       2024-10-28 2.1 加入前馈PID控制和无参构造函数
 *       2024-11-12 2.2 加入修改步进式PID的步长函数
 *       2024-11-22 2.3 在计算函数中如果反馈指针为空则真实值设置为0
 *       2025-1-10  3.0 新增串级pid类
 *       2025-10-28 3.1 修改命名
         2025-11-09 3.2 增加获取measure结构体接口
 * @example
 * 1.引用命名空间后，需要创建一个结构体 //PID_Init_Config_t pid_cof
 * 2.为其赋值，然后可以选择使用构造函数或者初始化函数对其初始化
 * 3.之后就可以直接调用计算方法ECF_PID_Calculate
 * 例程如下：
 *
     alg_n::PidInitConfig_t pid_cof = {
    .Kp = 1,
    .Ki = 1,
    .Kd = 0.5,
    .Kfa = 0,
    .Kfb = 0,
    .ActualValueSource = ptr,
    .mode = Deadzone | Integral_Limit | Derivative_On_Measurement | DerivativeFilter,
    .max_out = 2000,
    .max_Ierror = 1000,
    .gama = 0.1,
    .deadband = 10,
    .threshold_max = 2000,
    .threshold_min = -2000,
    .errorabsmax = 100,
    .errorabsmin = -100,
    .d_filter_num = 5,
    .out_filter_num = 10,
    .stepIn = 100
    };
    //不需要全部设置，根据所需设定需要的数值即可

    alg_n::PID_c pid_instance(pid_cof);
    pid_instance.Calc(SetValue); // 计算
    if (error || chance)
        pid_instance.Clear();             //清除
 *
 */

#include "Alg_PID.hpp"
using namespace alg_n;

/**
 * @brief          PID构造函数
 * @param[in]      pid_config pid 初始化参数结构体
 * @note 如想使用多个模式可以使用 ' | ',
 * @note 比如使用 积分分离 和 输出滤波 则可以在 mode 入参 (OutputFilter | Separated_Integral)
 */
PID_c::PID_c(PidInitConfig_t pid_config) : d_filter(pid_config.d_filter_num),
                                           out_filter(pid_config.out_filter_num)
{
    measure_={};
    if (pid_config.ActualValueSource != NULL)
        this->ActualValueSource_ = pid_config.ActualValueSource;
    else
        this->ActualValueSource_ = NULL;
    this->measure_.Kp = pid_config.Kp;
    this->measure_.Ki = pid_config.Ki;
    this->measure_.Kd = pid_config.Kd;
    this->measure_.D_T = pid_config.D_T;
    this->measure_.Kfa = pid_config.Kfa;
    this->measure_.Kfb = pid_config.Kfb;
    this->measure_.mode = pid_config.mode;

    this->measure_.max_Ierror = pid_config.max_Ierror;

    this->measure_.gama = pid_config.gama;

    this->measure_.threshold_max = pid_config.threshold_max;
    this->measure_.threshold_min = pid_config.threshold_min;

    this->measure_.max_out = pid_config.max_out;

    this->measure_.errorabsmax = pid_config.errorabsmax;
    this->measure_.errorabsmin = pid_config.errorabsmin;

    this->measure_.deadband = pid_config.deadband;

    this->measure_.stepIn = pid_config.stepIn;
    this->Clear();
}

// 无参构造函数
PID_c::PID_c() : d_filter(0), out_filter(0){}

/**
 * @brief          PID初始化函数,如果使用了构造就不需要这个函数了，这里主要用于初始化内部类
 * @param[in]      pid_config pid 初始化参数结构体
 * @note 如想使用多个模式可以使用 ' | ',
 * @note 比如使用 积分分离 和 输出滤波 则可以在 mode 入参 (OutputFilter | Separated_Integral)
 */
void PID_c::Init(PidInitConfig_t pid_config)
{
    measure_={};
    this->ActualValueSource_ = pid_config.ActualValueSource;
    this->measure_.Kp = pid_config.Kp;
    this->measure_.Ki = pid_config.Ki;
    this->measure_.Kd = pid_config.Kd;
    this->measure_.D_T = pid_config.D_T;
    this->measure_.Kfa = pid_config.Kfa;
    this->measure_.Kfb = pid_config.Kfb;
    this->measure_.mode = pid_config.mode;

    this->measure_.max_Ierror = pid_config.max_Ierror;

    this->measure_.gama = pid_config.gama;

    this->measure_.threshold_max = pid_config.threshold_max;
    this->measure_.threshold_min = pid_config.threshold_min;

    this->measure_.max_out = pid_config.max_out;

    this->out_filter.Init(pid_config.out_filter_num);

    this->measure_.errorabsmax = pid_config.errorabsmax;
    this->measure_.errorabsmin = pid_config.errorabsmin;

    this->d_filter.Init(pid_config.d_filter_num);

    this->measure_.deadband = pid_config.deadband;

    this->measure_.stepIn = pid_config.stepIn;
    this->Clear();
}

/**
 * @brief 更改闭环的实际值来源
 * @param ActValSource
 */
void PID_c::ChangeActValSource(float *ActValSource)
{
    this->ActualValueSource_ = ActValSource;
}

/**
 * @brief          PID清除
 * @retval         none
 * @attention      只是清除所有计算的数据，不会清除pid或者模式的数据
 */
void PID_c::Clear(void)
{
    this->measure_.error = this->measure_.LastError = 0.0f;
    this->measure_.Derror = this->measure_.LastDerror = this->measure_.LastLastDerror = 0.0f;
    this->measure_.out = this->measure_.Pout = this->measure_.Iout = this->measure_.Dout = this->measure_.Ierror = this->measure_.Fout = 0.0f;
    this->measure_.ActualValue = this->measure_.SetValue = this->measure_.LastActualValue = this->measure_.LastSetValue = this->measure_.PerrSetValue = 0.0f;
    this->d_filter.Clear();
    this->out_filter.Clear();
    pid_input = 0;
    pid_output = 0;
}

/**
 * @brief          PID通用计算
 * @param[in]      设定参考值
 * @retval         none
 */
float PID_c::Calc(float SetValuew)
{
    pid_input = SetValuew;
    float thiserr = 0; // 积分增值

    this->measure_.SetValue = SetValuew;

    // 步进式pid
    if (this->measure_.mode & static_cast<uint32_t>(StepIn))
        this->f_StepIn();

    if (this->ActualValueSource_ == NULL)
        this->measure_.ActualValue = 0;
    else
        this->measure_.ActualValue = *this->ActualValueSource_;

    this->measure_.error = this->measure_.SetValue - this->measure_.ActualValue;

    this->measure_.Derror = (this->measure_.error - this->measure_.LastError)/this->measure_.D_T;
    if (abs(this->measure_.error) >= this->measure_.deadband) // 死区
    {
        this->measure_.Pout = this->measure_.error * this->measure_.Kp;
        // 梯形积分
        if (this->measure_.mode & static_cast<uint32_t>(Trapezoid_integral))
            thiserr = (this->measure_.error + this->measure_.LastError) / 2;
        else
            thiserr = this->measure_.error * measure_.D_T;

        // 微分先行
        if (this->measure_.mode & static_cast<uint32_t>(Derivative_On_Measurement))
            this->f_Derivative_On_Measurement();
        else
            this->measure_.Dout = this->measure_.Kd * this->measure_.Derror;

        // 变积分
        if (this->measure_.mode & static_cast<uint32_t>(ChangingIntegrationRate))
            this->measure_.Ierror += thiserr * this->Changing_Integration_Rate();
        else
            this->measure_.Ierror += thiserr;

        // 积分限幅
        if (this->measure_.mode & static_cast<uint32_t>(Integral_Limit))
            this->f_Integral_Limit();
        this->measure_.Iout = this->measure_.Ki * this->measure_.Ierror;

        // 积分分离 注意需要放在iout计算后
        if (this->measure_.mode & static_cast<uint32_t>(Separated_Integral))
            this->f_Separated_Integral();

        // 微分滤波
        if (this->measure_.mode & static_cast<uint32_t>(DerivativeFilter))
            this->measure_.Dout = this->d_filter.Calc(this->measure_.Dout);

        // 前馈控制器
        if (this->measure_.mode & static_cast<uint32_t>(Feedforward))
            this->f_feedforward();

        this->measure_.out = this->measure_.Pout + this->measure_.Iout + this->measure_.Dout + this->measure_.Fout;

        // 输出滤波
        if (this->measure_.mode & static_cast<uint32_t>(OutputFilter))
            this->measure_.out = this->out_filter.Calc(this->measure_.out);

        // 输出限幅
        if (this->measure_.mode & static_cast<uint32_t>(Output_Limit))
            this->f_Output_Limit();
    }
    else
    {
        this->Clear();
    }

    // 数据更新
    this->measure_.LastActualValue = this->measure_.ActualValue;
    this->measure_.PerrSetValue = this->measure_.LastSetValue;
    this->measure_.LastSetValue = this->measure_.SetValue;
    this->measure_.LastDerror = this->measure_.Derror;
    this->measure_.LastError = this->measure_.error;

    this->pid_output = this->measure_.out;
    return this->pid_output;
}

/**
 * @brief          PID通用计算
 * @param[in]      设定参考值
 * @retval         none
 */
float PID_c::Calc(float SetValuew, float ActualValue)
{
    float thiserr = 0; // 积分增值

    this->measure_.SetValue = SetValuew;

    // 步进式pid
    if (this->measure_.mode & static_cast<uint32_t>(StepIn))
        this->f_StepIn();

    this->measure_.ActualValue = ActualValue;

    this->measure_.error = this->measure_.SetValue - this->measure_.ActualValue;

    this->measure_.Derror = (this->measure_.error - this->measure_.LastError )/this->measure_.D_T;
    if (abs(this->measure_.error) >= this->measure_.deadband) // 死区
    {
        this->measure_.Pout = this->measure_.error * this->measure_.Kp;
        // 梯形积分
        if (this->measure_.mode & static_cast<uint32_t>(Trapezoid_integral))
            thiserr = (this->measure_.error + this->measure_.LastError) / 2;
        else
            thiserr = this->measure_.error*this->measure_.D_T;

        // 微分先行
        if (this->measure_.mode & static_cast<uint32_t>(Derivative_On_Measurement))
            this->f_Derivative_On_Measurement();
        else
            this->measure_.Dout = this->measure_.Kd * this->measure_.Derror;

        // 变积分
        if (this->measure_.mode & static_cast<uint32_t>(ChangingIntegrationRate))
            this->measure_.Ierror += thiserr * this->Changing_Integration_Rate();
        else
            this->measure_.Ierror += thiserr;

        // 积分限幅
        if (this->measure_.mode & static_cast<uint32_t>(Integral_Limit))
            this->f_Integral_Limit();
        this->measure_.Iout = this->measure_.Ki * this->measure_.Ierror;

        // 积分分离 注意需要放在iout计算后
        if (this->measure_.mode & static_cast<uint32_t>(Separated_Integral))
            this->f_Separated_Integral();

        // 微分滤波
        if (this->measure_.mode & static_cast<uint32_t>(DerivativeFilter))
            this->measure_.Dout = this->d_filter.Calc(this->measure_.Dout);

        // 前馈控制器
        if (this->measure_.mode & static_cast<uint32_t>(Feedforward))
            this->f_feedforward();

        this->measure_.out = this->measure_.Pout + this->measure_.Iout + this->measure_.Dout + this->measure_.Fout;

        // 输出滤波
        if (this->measure_.mode & static_cast<uint32_t>(OutputFilter))
            this->measure_.out = this->out_filter.Calc(this->measure_.out);

        // 输出限幅
        if (this->measure_.mode & static_cast<uint32_t>(Output_Limit))
            this->f_Output_Limit();
    }
    else
    {
        this->Clear();
    }

    // 数据更新
    this->measure_.LastActualValue = this->measure_.ActualValue;
    this->measure_.PerrSetValue = this->measure_.LastSetValue;
    this->measure_.LastSetValue = this->measure_.SetValue;
    this->measure_.LastDerror = this->measure_.Derror;
    this->measure_.LastError = this->measure_.error;

    return this->measure_.out;
}

/**
 * @brief          积分分离
 * @retval         none
 * @note           error值超过阈值的时候把iout清零就行
 * @note           当设定值和参考值差别过大时, 只使用 PD 控制
 */
void PID_c::f_Separated_Integral(void)
{
    if (this->measure_.threshold_min > this->measure_.error || this->measure_.error > this->measure_.threshold_max)
        this->measure_.Iout = 0;
}

/**
 * @brief          积分限幅
 * @retval         none
 * @note      防止积分误差值(Ierror)超过限幅
 */
void PID_c::f_Integral_Limit(void)
{
    if (this->measure_.Ierror > this->measure_.max_Ierror)
    {
        this->measure_.Ierror = this->measure_.max_Ierror;
    }
    if (this->measure_.Ierror < -(this->measure_.max_Ierror))
    {
        this->measure_.Ierror = -(this->measure_.max_Ierror);
    }
}

/**
 * @brief          微分先行
 * @retval         none
 * @attention      //似乎存在问题 详见 https://blog.csdn.net/foxclever/article/details/80633275
 *                 问题似乎是原文c3符号的问题，评论区有争议，查阅其他资料是取负的，可参考：
 *https://wenku.baidu.com/view/17078d1bb91aa8114431b90d6c85ec3a87c28bbb?aggId=820bd2e1581b6bd97e19eacf&fr=catalogMain_text_ernie_recall_v1%3Awk_recommend_main1&_wkts_=1729588237892&bdQuery=%E5%BE%AE%E5%88%86%E5%85%88%E8%A1%8Cpid
 */
void PID_c::f_Derivative_On_Measurement(void)
{
    float c1, c2, c3, temp;

    temp = this->measure_.gama * this->measure_.Kd + this->measure_.Kp;
    c3 = this->measure_.Kd / temp;
    c2 = (this->measure_.Kd + this->measure_.Kp) / temp;
    c1 = this->measure_.gama * c3;
    this->measure_.Dout = c1 * this->measure_.Dout + c2 * this->measure_.ActualValue - c3 * this->measure_.LastActualValue;
}

/**
 * @brief           变积分系数处理函数，实现一个输出0和1之间的分段线性函数
 * @param[in]       pid结构体
 * @retval          积分增值系数
 * @note            当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
 * @note            当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间线性变化
 * @note            系统偏差大时，对积分作用减弱甚至是全无，而在偏差小时，加强积分的作用
 */
float PID_c::Changing_Integration_Rate(void)
{
    if (user_abs(this->measure_.error) <= this->measure_.errorabsmin) // 最小值
    {
        return 1.0f;
    }
    else if (user_abs(this->measure_.error) > this->measure_.errorabsmax) // 最大值
    {
        return 0.0f;
    }
    else
    {
        return (this->measure_.errorabsmax - user_abs(this->measure_.error)) / (this->measure_.errorabsmax - this->measure_.errorabsmin);
    }
}

/**
 * @brief          输出限幅
 * @param[in]      pid结构体
 * @retval         none
 * @attention      none
 */
void PID_c::f_Output_Limit(void)
{
    if (this->measure_.out > this->measure_.max_out)
    {
        this->measure_.out = this->measure_.max_out;
    }
    if (this->measure_.out < -(this->measure_.max_out))
    {
        this->measure_.out = -(this->measure_.max_out);
    }
}

/**
 * @brief           步进式pid
 * @retval          none
 * @attention       详见 https://blog.csdn.net/foxclever/article/details/81151898
 * @note            固定每周期的设定值变化值
 */
void PID_c::f_StepIn(void)
{
    float kFactor = 0.0f;
    if (abs(this->measure_.LastSetValue - this->measure_.SetValue) <= this->measure_.stepIn)
    {
        return;
    }
    else
    {
        if ((this->measure_.LastSetValue - this->measure_.SetValue) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((this->measure_.LastSetValue - this->measure_.SetValue) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        this->measure_.SetValue = this->measure_.LastSetValue + kFactor * this->measure_.stepIn;
    }
}

/**
 * @brief           前馈控制器
 * @param           SetValuew 当前设定值
 * @note            原理：求导数对未来状态进行预测
 */
void PID_c::f_feedforward(void)
{
    this->measure_.Fout = this->measure_.Kfa * (this->measure_.SetValue - this->measure_.LastSetValue) +
                          this->measure_.Kfb * (this->measure_.SetValue - 2 * this->measure_.LastSetValue + this->measure_.PerrSetValue);
}

const float *PID_c::GetActValSource(void)
{
    if (this->ActualValueSource_ != NULL)
        return this->ActualValueSource_;
    else
        return NULL;
}

//============================================串级PID=====================================================

// 串级PID构造函数
CascadePID_c::CascadePID_c(PidInitConfig_t outer_pid_config, PidInitConfig_t inter_pid_config, float output_limit) : Output_Limit(output_limit),outer_pid(outer_pid_config), inter_pid(inter_pid_config)
{
}

// 清除PID
void CascadePID_c::Clear()
{
    inter_pid.Clear();
    outer_pid.Clear();
}

// 串级pid计算函数（已传入反馈指针版
float CascadePID_c::Calc(const float *SetPosVal)
{
    Target = SetPosVal;
    // 计算外环
    outer_pid_output = (Outer_ActualVal == nullptr) ? outer_pid.Calc(*SetPosVal, 0) : outer_pid.Calc(*SetPosVal, *Outer_ActualVal);
    // 加入速度前馈
    if (SpeedFeedWard != nullptr)
        outer_pid_output += *SpeedFeedWard;
    // 计算内环
    inter_pid_output = (Inter_ActualVal == nullptr) ? inter_pid.Calc(outer_pid_output, 0) : inter_pid.Calc(outer_pid_output, *Inter_ActualVal);
    user_value_limit(inter_pid_output, -Output_Limit, Output_Limit);
    return inter_pid_output;
}

// 串级pid计算函数 （手动传值版
float CascadePID_c::Calc(const float *SetPosVal, const float *outer_ActValSource, const float *inter_ActValSource)
{
    Target = SetPosVal;
    // 计算外环
    outer_pid_output = (outer_ActValSource == nullptr) ? outer_pid.Calc(*SetPosVal, 0) : outer_pid.Calc(*SetPosVal, *outer_ActValSource);
    // 加入速度前馈
    if (SpeedFeedWard != nullptr)
        outer_pid_output += *SpeedFeedWard;
    // 计算内环
    inter_pid_output = (inter_ActValSource == nullptr) ? inter_pid.Calc(outer_pid_output, 0) : inter_pid.Calc(outer_pid_output, *inter_ActValSource);
    user_value_limit(inter_pid_output, -Output_Limit, Output_Limit);
    return inter_pid_output;
}

// 传入实际值指针
void CascadePID_c::SetValPoint(const float *outer_ActValSource, const float *inter_ActValSource)
{
    Outer_ActualVal = outer_ActValSource;
    Inter_ActualVal = inter_ActValSource;
}

// 传入速度前馈指针
void CascadePID_c::SetSpeedfeedward(const float *speed)
{
    SpeedFeedWard = speed;
}
