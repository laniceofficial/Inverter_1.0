#include <stdint.h>
#include "task_cpp.h"

#include "SEGGER_RTT.h"
#include "bsp_hrtim.hpp"
#include "pid.hpp"

#include "debug_capture.hpp"
#include "delaytrigger.hpp"
#include "sampling.hpp"

extern "C"
{
#include "gpio.h"
#include "lptim.h"
#include "main.h"
#include "tim.h"
    // #include "usart.h"
void SystemClock_Config(void);
}
// float test_times=0;
// float test = 0.9f;
float kChangingPowerTargetW = 60.0f;
float duty = 0.5f;

uint8_t enable = 0;
namespace
{

    constexpr uint16_t kAskProbeWindowTicks = 100U;
    constexpr float kAskDetectPowerTargetW = 35.0f;
    GPIO_TypeDef *const kChargeButtonPort = GPIOC;
    constexpr uint16_t kChargeButtonPin = GPIO_PIN_12;
    // Master + E/F 组成无线充电全桥波形。
    constexpr uint32_t kWirelessTimerMask = HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_E | HRTIM_TIMERID_TIMER_F;
    constexpr uint32_t kWirelessOutputMask = HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2 | HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2;

    // 发射端保护阈值（可根据实际硬件调整）
    constexpr float kTransmitterUvpThreshold = 15.0f;   // 输入欠压保护阈值 [V]
    constexpr float kTransmitterOvpThreshold = 30.0f;  // 输入过压保护阈值 [V]
    constexpr float kTransmitterOcpThreshold = 10.0f;  // 输入过流保护阈值 [A]

    // 根据当前 MCMP1/MCMP3 和 Timer E 的 CMP1/CMP3 计算出 TE1/TF1 同时导通的中心点，
    // 使 ADC 触发落在 DC 总线电流导通重叠区的正中间，避开开关边沿噪声。
    // void updateAdcTrigger()
    // {
    //     const uint32_t period = hhrtim1.Instance->sMasterRegs.MPER;
    //     if (period == 0U) { return; }

    //     // MCMP1 固定为 0，Timer E 导通窗口始终在 [CMP1E, CMP3E]。
    //     // ADC 触发放在 TE1 导通区的正中间，远离开关边沿。
    //     const uint32_t cmp1 = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR;
    //     const uint32_t cmp3 = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP3xR;

    //     const uint32_t trigger = (cmp1 + cmp3) / 2U;
    //     hhrtim1.Instance->sMasterRegs.MCMP2R = trigger;
    // }

    // 主任务对象：把低功耗探测、ASK 判定、PWM 波形和 HAL 回调集中管理。
    class WirelessChargeApp
    {
    public:
        void init();
        void loop();
        void dutyUpdate();
        void tryEnterLowPower();
        void onTimPeriodElapsed(TIM_HandleTypeDef *htim);
        void onLptimCompareMatch(LPTIM_HandleTypeDef *hlptim);
        void onGpioExti(uint16_t gpioPin);
        // void onUartRx(UART_HandleTypeDef *huart);
        void onAdcConvCplt(ADC_HandleTypeDef *hadc);
        ChangeState_e getNowState() const;
        void setNowState(ChangeState_e state);

    private:
        bool isChargeButtonPressed() const;
        void ensureTim6Started();
        void ensureTim6Stopped();
        void enterLowPowerMode();
        void startLowPowerProbe();
        void resumeActiveMode();
        void setNormalWaveform();
        void enableLowPowerProbe();
        void enableCharging();
        void disablePowerStage();
        void powerClosedLoop(float targetPowerW);
        void checkProtection(); // 检查故障并在触发时关断功率输出

        PID PIDpower;
        float phase = 0;
        ChangeState_e nowState_ = LOW_POWER;
        uint8_t tim6Running_ = 0U; // TIM6 是否正在驱动调制/任务节拍
        uint8_t lptimRunning_ = 0U; // LPTIM 是否正在低功耗唤醒计时
        uint8_t samplingRunning_ = 0U; // ADC 采样服务是否开启
        uint16_t probeTicks_ = 0U; // ASK 探测窗口剩余 tick

        // 保护用延时触发器（TIM6 1kHz 时基，每 tick = 1ms）
        Driver::DelayedTrigger uvpTrigger_;
        Driver::DelayedTrigger ovpTrigger_;
        Driver::DelayedTrigger ocpTrigger_;
        uint8_t faultActive_ = 0U; // 故障锁存，用于边沿触发关断/恢复
    };

    WirelessChargeApp &app()
    {
        static WirelessChargeApp instance;
        return instance;
    }

} // namespace

volatile DebugTraceEvent_t debug_trace_events[DEBUG_TRACE_EVENT_COUNT];
volatile uint16_t debug_trace_write_index = 0U;
volatile uint32_t debug_trace_seq = 0U;
volatile uint8_t flag = 1U;
// uint32_t cnt = 0U;
uint8_t uart_buf[20] = {};

void WirelessChargeApp::init()
{
    App::samplingService().init();
    pid_init(&PIDpower, PID_DELTA, 0.0f, 0.002f, 0.0f, 0.0, 0.5, -0.5f); // KP=0纯I, 消除ASK耦合

    SEGGER_RTT_Init();
    DebugCapture::init();
    samplingRunning_ = 1U;

    const Driver::HrtimConfig hrtimConfig = {
        .timerMask = kWirelessTimerMask,
        .period = 22666U,
        .phase = 0.0f,
        .masterRepetitionInterrupt = false,
    };
    Driver::configure(hrtimConfig);

    // 初始化保护触发器：25ms 确认 / 5ms 释放，过流用更长确认时间
    uvpTrigger_.init(25U, 2U);
    ovpTrigger_.init(25U, 2U);
    ocpTrigger_.init(50U, 2U);

    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_buf, 20);
    enableCharging();
    ensureTim6Started();
    nowState_ = ASKDetect;
}

void WirelessChargeApp::loop()
{
    checkProtection();

    // 状态机每次只处理当前状态的一小步，HAL 回调负责推进采样和计时。
    switch (nowState_)
    {
        case LOW_POWER:
            // enterLowPowerMode();
            return;

        case PreDetect:
            // SEGGER_RTT_TerminalOut(0, "ASK_DETECT\r\n");
            // startLowPowerProbe();
            return;

        case ASKDetect:
            powerClosedLoop(kAskDetectPowerTargetW);
            if (App::samplingService().isAskValid() &&
                App::samplingService().isRequirePower()) //&& isChargeButtonPressed())
            {
                resumeActiveMode();
                return;
            }

            if (probeTicks_ > 0U)
            {
                --probeTicks_;
            }

            if (probeTicks_ == 0U)
            {
                // enterLowPowerMode();
            }
            return;

        case PreChange:
            // SEGGER_RTT_TerminalOut(0, "START_CHARGE\r\n");
            if (!App::samplingService().isAskValid() ||
                !App::samplingService().isRequirePower()) //|| !isChargeButtonPressed())
            {
                // enterLowPowerMode();
                nowState_ = ASKDetect;
                return;
            }

            nowState_ = Changing;
            powerClosedLoop(kChangingPowerTargetW);
            return;

        case Changing:
            if (!App::samplingService().isAskValid() ||
                !App::samplingService().isRequirePower()) //|| !isChargeButtonPressed())
            {
                // enterLowPowerMode();
                nowState_ = ASKDetect;
                return;
            }

            powerClosedLoop(kChangingPowerTargetW);
            return;

        default:
            // SEGGER_RTT_TerminalOut(0, "ERROR\r\n");
            // enterLowPowerMode();
            return;
    }
}


void WirelessChargeApp::powerClosedLoop(const float targetPowerW)
{
    PIDpower.ref = targetPowerW;
    pid_calculate(&PIDpower, App::samplingService().getTransmitterPower());
    // DELTA PID 输出除以控制频率 → 等效纯 I 积分器，ASK 1kHz 纹波天然积零
    phase += PIDpower.output / 1000.0f;
    if (phase < 0.0f)
    {
        phase = 0.0f;
    }
    if (phase > 0.9f)
    {
        phase = 0.9f;
    }
    Driver::setPhase(phase);

    // updateAdcTrigger();
}

void WirelessChargeApp::tryEnterLowPower()
{
    if (nowState_ != LOW_POWER)
    {
        return;
    }

    if ((lptimRunning_ == 1U) && (nowState_ != LOW_POWER))
    {
        return;
    }

    if (lptimRunning_ == 0U)
    {
        lptimRunning_ = 1U;
    }

    HAL_SuspendTick();
    // STOP 唤醒后系统时钟会恢复到默认状态，需要重新执行 SystemClock_Config。
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    SystemClock_Config();
    HAL_ResumeTick();
}
void WirelessChargeApp::dutyUpdate()
{
    // Driver::setPhase(test);
    setNormalWaveform();
    // updateAdcTrigger();
}
void WirelessChargeApp::onTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        // dutyUpdate();

        loop();
        // if (enable)
        // {
        //     powerClosedLoop(kChangingPowerTargetW);
        // }
    }

    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
}

void WirelessChargeApp::onLptimCompareMatch(LPTIM_HandleTypeDef *hlptim)
{
    if (hlptim != &hlptim1)
    {
        return;
    }

    HAL_LPTIM_TimeOut_Stop_IT(hlptim);
    lptimRunning_ = 0U;
    nowState_ = PreDetect;
}

void WirelessChargeApp::onGpioExti(uint16_t)
{
    // 充电按键唤醒后直接准备进入正式充电流程。
    if (HAL_GPIO_ReadPin(kChargeButtonPort, kChargeButtonPin) == GPIO_PIN_SET)
    {
        if (lptimRunning_ != 0U)
        {
            resumeActiveMode();
            App::samplingService().start();
            lptimRunning_ = 0U;
        }

        nowState_ = PreChange;
        return;
    }

    else if (HAL_GPIO_ReadPin(F1KHZ_GPIO_Port, F1KHZ_Pin) == GPIO_PIN_SET)
    {
        flag = 1U;
        // App::ReceiverAsk::setDivider(1U);
    }
    else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
    {
        if (lptimRunning_ != 0U)
        {
            resumeActiveMode();
            App::samplingService().start();
            lptimRunning_ = 0U;
        }
        // test_times++;
        nowState_ = PreChange;
        // App::ReceiverAsk::setDivider(10U);
    }
}
// void WirelessChargeApp::onUartRx(UART_HandleTypeDef *huart)
// {
//     if (lptimRunning_ != 0U)
//     {
//         resumeActiveMode();
//         App::samplingService().start();
//         lptimRunning_ = 0U;
//     }

//     nowState_ = PreChange;
//     test++;
// }

void WirelessChargeApp::onAdcConvCplt(ADC_HandleTypeDef *hadc)
{
    App::samplingService().handleAdcConvCpltCallback(hadc);
}

ChangeState_e WirelessChargeApp::getNowState() const
{
    return nowState_;
}

void WirelessChargeApp::setNowState(const ChangeState_e state)
{
    nowState_ = state;
}

bool WirelessChargeApp::isChargeButtonPressed() const
{
    return HAL_GPIO_ReadPin(kChargeButtonPort, kChargeButtonPin) == GPIO_PIN_SET;
}

void WirelessChargeApp::ensureTim6Started()
{
    if (tim6Running_ == 0U)
    {
        HAL_TIM_Base_Start_IT(&htim6);
        tim6Running_ = 1U;
    }
}

void WirelessChargeApp::ensureTim6Stopped()
{
    if (tim6Running_ != 0U)
    {
        HAL_TIM_Base_Stop_IT(&htim6);
        tim6Running_ = 0U;
    }
}

void WirelessChargeApp::enterLowPowerMode()
{
    // 进入低功耗前必须先关功率输出和 ADC，避免 STOP 期间外设继续耗电。
    disablePowerStage();

    if (samplingRunning_ != 0U)
    {
        App::samplingService().stop();
        samplingRunning_ = 0U;
    }

    App::samplingService().resetAskValid();
    ensureTim6Stopped();
    probeTicks_ = 0U;
    nowState_ = LOW_POWER;
    tryEnterLowPower();
}

void WirelessChargeApp::startLowPowerProbe()
{
    // 低功耗探测只打开短时间采样和弱激励，窗口结束未检测到 ASK 则回到 STOP。
    App::samplingService().resetAskValid();
    App::samplingService().start();
    samplingRunning_ = 1U;
    enableLowPowerProbe();
    ensureTim6Started();
    probeTicks_ = kAskProbeWindowTicks;
    nowState_ = ASKDetect;
}

void WirelessChargeApp::resumeActiveMode()
{
    // enableCharging();
    // ensureTim6Started();
    probeTicks_ = 0U;
    nowState_ = PreChange;
}

void WirelessChargeApp::setNormalWaveform()
{
    Driver::setDuty(Driver::HrtimTimer::TimerE, duty);
    Driver::setDuty(Driver::HrtimTimer::TimerF, duty);
}

void WirelessChargeApp::enableLowPowerProbe()
{
    Driver::startTimers(kWirelessTimerMask);
    Driver::enableOutputs(kWirelessOutputMask);

    // 探测模式使用固定比较值输出较弱激励，避免一开始就全功率运行。
    Driver::setCompare(Driver::HrtimTimer::TimerE, HRTIM_COMPAREUNIT_1, 13600U);
    Driver::setCompare(Driver::HrtimTimer::TimerE, HRTIM_COMPAREUNIT_3, 40800U);
    Driver::setCompare(Driver::HrtimTimer::TimerF, HRTIM_COMPAREUNIT_1, 13600U);
    Driver::setCompare(Driver::HrtimTimer::TimerF, HRTIM_COMPAREUNIT_3, 40800U);
}

void WirelessChargeApp::enableCharging()
{
    Driver::startTimers(kWirelessTimerMask);
    Driver::setPhase(0.0);
    Driver::enableOutputs(kWirelessOutputMask);
    setNormalWaveform();
    hhrtim1.Instance->sMasterRegs.MCMP2R = 16000;
}

void WirelessChargeApp::disablePowerStage()
{
    Driver::disableOutputs(kWirelessOutputMask);
    Driver::stopTimers(kWirelessTimerMask);
}

void WirelessChargeApp::checkProtection()
{
    const float voltage = App::samplingService().getTransmitterVoltage();
    const float current = App::samplingService().getTransmitterCurrent();

    // 原始故障判定（阈值比较）
    const bool uvpRaw = (voltage < kTransmitterUvpThreshold);
    const bool ovpRaw = (voltage > kTransmitterOvpThreshold);
    const bool ocpRaw = (current > kTransmitterOcpThreshold);

    // 经延时触发器消抖后确认故障
    const bool uvpFault = uvpTrigger_.update(uvpRaw);
    const bool ovpFault = ovpTrigger_.update(ovpRaw);
    const bool ocpFault = ocpTrigger_.update(ocpRaw);
    const bool anyFault = (uvpFault || ovpFault || ocpFault);

    if (anyFault && (faultActive_ == 0U))
    {
        // 上升沿：故障刚触发，关断输出
        faultActive_ = 1U;
        disablePowerStage();
        pid_reset(&PIDpower);
        phase = 0.0f;
    }
    else if (!anyFault && (faultActive_ != 0U))
    {
        // 下降沿：故障已消除，恢复输出
        faultActive_ = 0U;
        // if (nowState_ == ASKDetect)
        // {
        //     enableLowPowerProbe();
        // }
        // else
        // {
            enableCharging();
        // }
    }
}

extern "C" void task_init(void)
{
    app().init();
}

extern "C" void task_loop(void)
{
    app().loop();
}

extern "C" void duty_update(void)
{
    app().dutyUpdate();
}

extern "C" void task_try_enter_low_power(void)
{
    app().tryEnterLowPower();
}

extern "C" ChangeState_e GetNowState(void)
{
    return app().getNowState();
}

extern "C" void SetNowState(ChangeState_e state)
{
    app().setNowState(state);
}

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
    // 该函数可能在中断中调用，写环形缓冲时短暂关中断保证索引一致。
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

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    app().onAdcConvCplt(hadc);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    app().onTimPeriodElapsed(htim);
    DebugCapture::onTimPeriodElapsed(htim);
}

extern "C" void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
    app().onLptimCompareMatch(hlptim);
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t gpioPin)
{
    app().onGpioExti(gpioPin);
}

// extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart == &huart2)
//     {
//         app().onUartRx(huart);
//     }
// }

// extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t)
// {
//     if (huart == &huart2)
//     {

//         app().onUartRx(huart);
//         }
// }
