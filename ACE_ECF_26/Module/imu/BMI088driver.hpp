#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

// #include "gpio.h"
#include "bsp_PWM.hpp"
#include "quaternion_EKF.hpp"
#include "Alg_PID.hpp"
#include <cstdint>

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

typedef enum
{
    X = 0,
    Y = 1,
    Z = 2,
} INS_idx_e;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/* 六轴原始数据结构体 */
typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];

    float gNorm;
} IMU_Data_t;

/* 状态解算数据结构体 */
typedef struct ins
{
public:
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;

/* BMI088错误码枚举 */
enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

extern IMU_Data_t BMI088; // 外部用到姿态解算算法时extern出去

// imu 通用加热类
typedef enum
{
    IMU_Heat_Uninit = 0,         // 未初始化
    IMU_Heat_Inited,             // 已初始化
    IMU_Heat_PreHeating,         // 预热中
    IMU_Heat_PreHeatFail,        // 预热失败
    IMU_Heat_PreHeat_waitSteady, // 加热中,满功率预热结束，等待温度稳定
    IMU_Heat_OK,                 // 预热成功,正常加热
    IMU_Heat_OverTemp,           // 过温保护

} IMU_Heat_Status_enum_t; // imu加热器加热状态

typedef struct
{
    TIM_HandleTypeDef *htimx;
    uint32_t channel;

    float max_temp;           // 最高温度,达到该温度视为热失控，强制关闭加热输出，直到降低到初始化温度
    float init_temp;          // 初始化温度
    float target_temp;        // 目标温度
    uint32_t preheat_TimeOut; // 预热超时时间，单位ms

    alg_n::PidInitConfig_t *pid_config; // 加热器pid参数配置
    uint8_t heater_forward;             // 加热器固定前馈值
} IMU_Heat_Config_s_t;                  // IMU加热器配置

class BMI088Heat_c;

class BMI088_c
{
public:
    uint8_t init_flag = 0;
    BMI088_c();
    uint8_t physicalInit(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate,
                         GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                         uint16_t _ACCEL_CS_Pin,
                         GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                         uint16_t _GYRO_CS_Pin);
    uint8_t CalibrateMpuOffset();
    uint8_t isImuInited();
    int16_t caliCount = 0;  // 调试用的变量
    uint8_t caliOffset = 1; // 是否开启较准零飘

    float gyroDiff[3], gNormDiff;
    IMU_Data_t BMI088; // 六轴原始数据
    INS_t INS;         // 状态解算数据
    IMU_Param_t IMU_Param;
    static void BMI_UpData(void);          // 更新陀螺仪数据
    float ReadTemp(void);                  // 读取温度
    const INS_t *Get_INS_Data_Point(void); // 返回陀螺仪数据结构体
    QEKF_INS_t QEKF_INS;                   // EKF滤波结构体
    static BMI088_c *BMI_List[10];

private: // 私有变量
    SPI_HandleTypeDef *BMI088_WHO;
    // 用于获取两次采样之间的时间间隔
    float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
    float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    uint32_t INS_DWT_Count = 0;
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t error = BMI088_NO_ERROR;
    void BMI088_Read(void);
    void Attitude_Calc(void);
    uint8_t bmi088_accel_init(void);
    uint8_t bmi088_gyro_init(void);
    void Calibrate_MPU_Offset();
    void IMU_Param_Correction();
    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF);
    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF);
    void InitQuaternion(float *init_q4);

    uint8_t add_flag = 0;

    // 陀螺仪引脚交互部分
    GPIO_TypeDef *_ACCEL_CS_GPIO_Port; // 加速度计片选引脚端口
    uint16_t _ACCEL_CS_Pin;            // 加速度计片选引脚号
    GPIO_TypeDef *_GYRO_CS_GPIO_Port;  // 陀螺仪计片选引脚端口
    uint16_t _GYRO_CS_Pin;             // 陀螺仪计片选引脚号
    void BMI088_ACCEL_NS_L(void);
    void BMI088_ACCEL_NS_H(void);
    void BMI088_GYRO_NS_L(void);
    void BMI088_GYRO_NS_H(void);
    void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
    uint8_t BMI088_read_write_byte(uint8_t txdata);
    void BMI088_write_single_reg(uint8_t reg, uint8_t data);
    void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
};

class BMI088Heat_c : public BMI088_c
{
public:
    BMI088Heat_c(IMU_Heat_Config_s_t *_configP); // 对数据初始化

    void timInit();                            // 开定时器，需要在系统启动后
    void setTemp(float _target_temp);          // 设置目标温度
    void heatLoop();                           // imu加热主循环
    void preHeat();                            // 预热
    IMU_Heat_Status_enum_t getHeatingStatus(); // 获取加热状态
    void stopheat() { heater_PWM.PWMSetDuty((uint8_t)0); }

  private:
    IMU_Heat_Config_s_t config;
    IMU_Heat_Status_enum_t status;
    alg_n::PID_c *heat_PID;
    BSP_n::PWM_c heater_PWM;
};

#endif