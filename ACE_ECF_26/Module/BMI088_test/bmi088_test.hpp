#ifndef bmi088_test_hpp
#define bmi088_test_hpp

// #include "main.h"
#include "bsp_spi.hpp"
#include "alg_quaternion.hpp"
#include "alg_filter_ekf.hpp"




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
// 需手动修改
#if INFANTRY_ID == 0
#define GxOFFSET 0.00247530174f
#define GyOFFSET 0.000393082853f
#define GzOFFSET 0.000393082853f
#define gNORM 9.69293118f
#elif INFANTRY_ID == 1
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 2
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#elif INFANTRY_ID == 3
#define GxOFFSET 0.00270364084f
#define GyOFFSET -0.000532632112f
#define GzOFFSET 0.00478090625f
#define gNORM 9.73574924f
#elif INFANTRY_ID == 4
#define GxOFFSET 0.0007222f
#define GyOFFSET -0.001786f
#define GzOFFSET 0.0004346f
#define gNORM 9.876785f
#endif
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
/* 六轴原始数据结构体 */
typedef struct
{
    float Accel[3];

    float Gyro[3];

    float TempWhenCali;
    float Temperature;

    float AccelScale;
    float GyroOffset[3];
    float gNormDiff;
    float gyroDiff[3];
    float gNorm;
} IMU_Data_t;
class T_BMI088_c
{
public:
    T_BMI088_c();
    uint8_t BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate,
                       GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                       uint16_t _ACCEL_CS_Pin,
                       GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                       uint16_t _GYRO_CS_Pin);
    int16_t caliCount = 0;  // 调试用的变量
    uint8_t caliOffset = 1; // 是否开启较准零飘
    IMU_Data_t RawDate;      // 六轴原始数据
    SPI_HandleTypeDef *BMI088_WHO;
    static void BMI_UpData(void);          // 更新陀螺仪数据
    Class_Filter_EKF<4, 3, 3> EKF_Quaternion;
    // EKF的相关函数
    // 四元数状态转移函数
    static Class_Matrix_f32<4, 1> EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对状态的雅可比矩阵
    static Class_Matrix_f32<4, 4> EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对过程噪声的雅可比矩阵
    static Class_Matrix_f32<4, 3> EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数测量函数
    static Class_Matrix_f32<3, 1> EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对状态的雅可比矩阵
    static Class_Matrix_f32<3, 4> EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对测量噪声的雅可比矩阵
    static Class_Matrix_f32<3, 3> EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);
    void Accel_Chi_Square_Calculate();

    inline float Get_Angle_Yaw() const;

    inline float Get_Angle_Pitch() const;

    inline float Get_Angle_Roll() const;

    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;

    inline float Get_Rodrigues_Angle() const;

    inline Class_Matrix_f32<3, 1> Get_Rodrigues_Axis() const;

    inline Class_Quaternion_f32 Get_Quaternion() const;

    inline float Get_Accel_Chi_Square_Loss() const;

    inline uint64_t Get_Calculating_Time() const;

private: // 私有变量
         // 四元数
    Class_Quaternion_f32 Quaternion;
    // EKF计算时间戳
    uint64_t EKF_Now_Timestamp = 0;
    // 上次EKF计算时间戳
    uint64_t EKF_Pre_Timestamp = 0;

    // 时间差
    float D_T = 0.001005f;
    // 卡方检验值
    float Accel_Chi_Square_Loss = 0.0f;
    // 卡方检验残差阈值
    float ACCEL_CHI_SQUARE_TEST_THRESHOLD = 3.0f;
    // 加速度计源数据
    Class_Matrix_f32<3, 1> Vector_Original_Accel;
    // 加速度计归一化数据
    Class_Matrix_f32<3, 1> Vector_Normalized_Accel;
    // 陀螺仪源数据
    Class_Matrix_f32<3, 1> Vector_Original_Gyro;
    // 上一次陀螺仪源数据
    Class_Matrix_f32<3, 1> Vector_Pre_Original_Gyro; 
    // 欧拉角, Yaw-Pitch-Roll顺序
    Class_Matrix_f32<3, 1> Vector_Euler_Angle;
    // 旋转矩阵
    Class_Matrix_f32<3, 3> Matrix_Rotation;
    // 轴角式
    Class_Matrix_f32<4, 1> Vector_Axis_Angle;
    // 处理时间
    uint64_t Calculating_Time = 0;
    // 数据更新标志
    bool Accel_Update_Flag = false;
    uint64_t Accel_Update_Timestamp = 0;
    bool Gyro_Update_Flag = false;
    uint64_t Gyro_Update_Timestamp = 0;
    // 数据合法标志
    bool Accel_Valid_Flag = false;
    bool Gyro_Valid_Flag = false;

    // 用于获取两次采样之间的时间间隔
    float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
    float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    uint8_t res = 0;
    uint8_t write_reg_num = 0;
    uint8_t error = BMI088_NO_ERROR;
    void BMI088_Read(void);
    void Attitude_Calc(void);
    uint8_t bmi088_accel_init(void);
    uint8_t bmi088_gyro_init(void);
    void Calibrate_MPU_Offset();
    void InitQuaternion();

    uint8_t add_flag = 0;
    static T_BMI088_c *BMI_List[1];

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

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取偏航角
 *
 * @return 偏航角, 单位rad
 */
inline float T_BMI088_c::Get_Angle_Yaw() const
{
    return (Vector_Euler_Angle[0][0]);
}

/**
 * @brief 获取俯仰角
 *
 * @return 俯仰角, 单位rad
 */
inline float T_BMI088_c::Get_Angle_Pitch() const
{
    return (Vector_Euler_Angle[1][0]);
}

/**
 * @brief 获取横滚角
 *
 * @return 横滚角, 单位rad
 */
inline float T_BMI088_c::Get_Angle_Roll() const
{
    return (Vector_Euler_Angle[2][0]);
}

/**
 * @brief 获取旋转矩阵
 *
 */
inline Class_Matrix_f32<3, 3> T_BMI088_c::Get_Rotation_Matrix() const
{
    return (Matrix_Rotation);
}

/**
 * @brief 获取轴角式的角度
 *
 */
inline float T_BMI088_c::Get_Rodrigues_Angle() const
{
    return (Vector_Axis_Angle[0][0]);
}

#endif // !bmi088_test_hpp

