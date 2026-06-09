#define TransmitterVoltageGain 0.00887f
#define TransmitterVoltageBias 0.64416f
#define TransmitterCurrentGain 0.0015f
#define TransmitterCurrentBias -0.3028f
// 发射端 ADC3 标定公式：工程量 = (raw平均值* gain+ bias)
#define kVoltageRef 3.306f
#define kAdcMaxValue 4095.0f
