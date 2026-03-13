#include "bmi088_test.hpp"
#include "BMI088reg_test.hpp"
#include "bsp_dwt.hpp"
// 类外初始化
T_BMI088_c *T_BMI088_c::BMI_List[1] = {nullptr};
static BSP_n::DWT_c *dwt_bmi088 = BSP_n::DWT_c::Get_DwtInstance();
#define BMI088_USE_SPI

#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

#elif defined(BMI088_USE_IIC)
#endif
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
// test BMI088_GYRO_1000_116_HZ  BMI088_ACC_800_HZ BMI088_GYRO_2000 测试开摩擦轮时会不会减少picth抖动,,
// 已减小
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {

        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, // 改动会导致计算延缓
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
 * @brief 校准零飘，向BMI_List添加本陀螺仪
 */
T_BMI088_c::T_BMI088_c()
{
}
/**
 * @brief 所有陀螺仪数值、姿态更新
 */
void T_BMI088_c::BMI_UpData(void)
{
    uint8_t BMI_List_Idx = 0;
    while (BMI_List_Idx < (sizeof(BMI_List) / sizeof(BMI_List[0])) && BMI_List[BMI_List_Idx] != nullptr)
    {
       float in = dwt_bmi088->GetTimeline_ms();
        BMI_List[BMI_List_Idx]->BMI088_Read();
        BMI_List[BMI_List_Idx]->Attitude_Calc();
        BMI_List_Idx++;
        float out = dwt_bmi088->GetTimeline_ms();
        float delta_t = out - in; //0.876ms
    }
}
void T_BMI088_c::Attitude_Calc(void)
{
    EKF_Now_Timestamp = dwt_bmi088->GetTimeline_us();
    // 设置时间差
    D_T = (EKF_Now_Timestamp - EKF_Pre_Timestamp) / 1000000.0f;
    EKF_Quaternion.Set_D_T(D_T);

    // EKF预测
    EKF_Quaternion.Vector_U = Vector_Original_Gyro;
    EKF_Quaternion.TIM_Predict_PeriodElapsedCallback();

    // EKF更新
    if (Accel_Update_Flag && Accel_Valid_Flag)
    {
        Accel_Chi_Square_Calculate();
        if (Accel_Chi_Square_Loss <= ACCEL_CHI_SQUARE_TEST_THRESHOLD)
        {
            // 卡方检验通过, 更新
            EKF_Quaternion.Vector_Z = Vector_Normalized_Accel;
            EKF_Quaternion.TIM_Update_PeriodElapsedCallback();
        }
        // 加速度已利用过, 清除标志
        Accel_Update_Flag = false;
    }

    // x归一化, 按理来说这也算模型的一部分, 应当放到系统函数F中, 且需要更新Jacobi矩阵
    // 然而实测发现是否更新对性能影响不算太大, 更新反而占用了计算时间
    EKF_Quaternion.Vector_X = EKF_Quaternion.Vector_X.Get_Normalization();

    // 数据输出

    Quaternion = EKF_Quaternion.Vector_X;

    Vector_Euler_Angle = Quaternion.Get_Euler_Angle();
    Matrix_Rotation = Quaternion.Get_Rotation_Matrix();
    Vector_Axis_Angle = Quaternion.Get_Rodrigues();

    Calculating_Time = dwt_bmi088->GetTimeline_us()- EKF_Now_Timestamp;

    EKF_Pre_Timestamp = EKF_Now_Timestamp;
    Vector_Pre_Original_Gyro = Vector_Original_Gyro;
}

/**
 * @brief 读取BMI088的原始六轴数据
 */
void T_BMI088_c::BMI088_Read(void)
{

    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;
    // 轴加速度计原始数据读取
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    RawDate.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    RawDate.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    RawDate.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    // 角加速度计原始数据读取
    Accel_Update_Flag = true;
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            RawDate.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            RawDate.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            RawDate.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            RawDate.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            RawDate.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            RawDate.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    Vector_Original_Accel[0][0] = RawDate.Accel[0];
    Vector_Original_Accel[1][0] = RawDate.Accel[1];
    Vector_Original_Accel[2][0] = RawDate.Accel[2];

    Vector_Original_Gyro[0][0] = RawDate.Gyro[0];
    Vector_Original_Gyro[1][0] = RawDate.Gyro[1];
    Vector_Original_Gyro[2][0] = RawDate.Gyro[2];
    RawDate.Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    // 防止NaN流入算法, 加速度计数据不合法直接丢弃, 陀螺仪数据不合法则使用上次数据

    Vector_Normalized_Accel = Vector_Original_Accel.Get_Normalization();
    Accel_Valid_Flag = true;
    if (alg_n::IsInvalid_loat(Vector_Original_Accel[0][0]) || alg_n::IsInvalid_loat(Vector_Original_Accel[1][0]) || alg_n::IsInvalid_loat(Vector_Original_Accel[2][0]))
    {
        Accel_Valid_Flag = false;
    }
    Gyro_Valid_Flag = true;
    if (alg_n::IsInvalid_loat(Vector_Original_Gyro[0][0]) || alg_n::IsInvalid_loat(Vector_Original_Gyro[1][0]) || alg_n::IsInvalid_loat(Vector_Original_Gyro[2][0]))
    {
        Gyro_Valid_Flag = false;
        Vector_Original_Gyro = Vector_Pre_Original_Gyro;
    }
}
uint8_t T_BMI088_c::BMI088Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate,
                             GPIO_TypeDef *Parm_ACCEL_CS_GPIO_Port,
                             uint16_t Parm_ACCEL_CS_Pin,
                             GPIO_TypeDef *Parm_GYRO_CS_GPIO_Port,
                             uint16_t Parm_GYRO_CS_Pin)
{
    // 强制复位两条片选引脚，防止笨b配错cubemx的片选引脚高低状态
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    this->BMI088_WHO = bmi088_SPI;
    this->_ACCEL_CS_GPIO_Port = Parm_ACCEL_CS_GPIO_Port;
    this->_ACCEL_CS_Pin = Parm_ACCEL_CS_Pin;
    this->_GYRO_CS_GPIO_Port = Parm_GYRO_CS_GPIO_Port;
    this->_GYRO_CS_Pin = Parm_GYRO_CS_Pin;
    // this->ACCEL_CS_GPIO_Port = CS1_ACCEL_GPIO_Port;
    // this->ACCEL_CS_Pin = CS1_ACCEL_Pin;
    // this->GYRO_CS_GPIO_Port = CS1_GYRO_GPIO_Port;
    // this->GYRO_CS_Pin = CS1_GYRO_Pin;

    if (add_flag == 0)
    {
        add_flag = 1;
        uint8_t BMI_List_Idx = 0;
        while (BMI_List[BMI_List_Idx] != nullptr && BMI_List_Idx < (sizeof(BMI_List) / sizeof(BMI_List[0])))
            BMI_List_Idx++; // 越界风险
        BMI_List[BMI_List_Idx] = this;
    }
    do
    {
        this->error = BMI088_NO_ERROR;
        this->error |= bmi088_accel_init();
        this->error |= bmi088_gyro_init();
        if (calibrate)
            Calibrate_MPU_Offset();
        else
        {
            RawDate.GyroOffset[0] = GxOFFSET;
            RawDate.GyroOffset[1] = GyOFFSET;
            RawDate.GyroOffset[2] = GzOFFSET;
            RawDate.gNorm = gNORM;
            RawDate.AccelScale = 9.81f / RawDate.gNorm;
            RawDate.TempWhenCali = 40;
        }
    } while (this->error != BMI088_NO_ERROR);
    BMI088_Read();
    InitQuaternion();
    return this->error;
}
/**
 * @brief 校准零飘
 */
void T_BMI088_c::Calibrate_MPU_Offset()
{
    static float startTime;
    static uint16_t CaliTimes = 6000; // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;

    startTime = dwt_bmi088->GetTimeline_s();
    do
    {
        if (dwt_bmi088->GetTimeline_s() - startTime > 12)
        {
            // 校准超时
            RawDate.GyroOffset[0] = GxOFFSET;
            RawDate.GyroOffset[1] = GyOFFSET;
            RawDate.GyroOffset[2] = GzOFFSET;
            RawDate.gNorm = gNORM;
            RawDate.TempWhenCali = 40;
            break;
        }
        dwt_bmi088->Delay_s(0.005);
        RawDate.gNorm = 0;
        RawDate.GyroOffset[0] = 0;
        RawDate.GyroOffset[1] = 0;
        RawDate.GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; ++i)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            RawDate.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            RawDate.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            RawDate.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(RawDate.Accel[0] * RawDate.Accel[0] +
                              RawDate.Accel[1] * RawDate.Accel[1] +
                              RawDate.Accel[2] * RawDate.Accel[2]);
            RawDate.gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                RawDate.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                RawDate.GyroOffset[0] += RawDate.Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                RawDate.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                RawDate.GyroOffset[1] += RawDate.Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                RawDate.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
                RawDate.GyroOffset[2] += RawDate.Gyro[2];
            }
            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    gyroMax[j] = RawDate.Gyro[j];
                    gyroMin[j] = RawDate.Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; ++j)
                {
                    if (RawDate.Gyro[j] > gyroMax[j])
                        gyroMax[j] = RawDate.Gyro[j];
                    if (RawDate.Gyro[j] < gyroMin[j])
                        gyroMin[j] = RawDate.Gyro[j];
                }
            }
            // 数据差异过大认为收到外界干扰，需重新校准
            RawDate.gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; ++j)
                RawDate.gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (RawDate.gNormDiff > 0.5f ||
                RawDate.gyroDiff[0] > 0.15f ||
                RawDate.gyroDiff[1] > 0.15f ||
                RawDate.gyroDiff[2] > 0.15f)
            {
                break;
            }
            dwt_bmi088->Delay_s(0.0005);
        }
        // 取平均值得到标定结果
        RawDate.gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; ++i)
            RawDate.GyroOffset[i] /= (float)CaliTimes;
        // 记录标定时IMU温度
        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        RawDate.TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        this->caliCount++;
    } while (RawDate.gNormDiff > 0.5f ||
             fabsf(RawDate.gNorm - 9.8f) > 0.5f ||
             RawDate.gyroDiff[0] > 0.15f ||
             RawDate.gyroDiff[1] > 0.15f ||
             RawDate.gyroDiff[2] > 0.15f ||
             fabsf(RawDate.GyroOffset[0]) > 0.01f ||
             fabsf(RawDate.GyroOffset[1]) > 0.01f ||
             fabsf(RawDate.GyroOffset[2]) > 0.01f);
    // 根据标定结果校准加速度计标度因数
    RawDate.AccelScale = 9.81f / RawDate.gNorm;
}
void T_BMI088_c::InitQuaternion()
{
    EKF_Now_Timestamp = dwt_bmi088->GetTimeline_us();
    // 过程噪声协方差矩阵
    float array_q[9] = {0.865f, 0.0f, 0.0f, 0.0f, 0.975f, 0.0f, 0.0f, 0.0f, 1.077f};
    Class_Matrix_f32<3, 3> matrix_q(array_q);
    // 测量噪声协方差矩阵
    float array_r[9] = {0.0446f, 0.0f, 0.0f, 0.0f, 0.0476f, 0.0f, 0.0f, 0.0f, 0.0537f};
    Class_Matrix_f32<3, 3> matrix_r(array_r);
    // 初始状态协方差矩阵
    Class_Matrix_f32<4, 4> matrix_p = Namespace_ALG_Matrix::Identity<4, 4>();
    // 初始状态向量
    Class_Matrix_f32<4, 1> vector_x = Namespace_ALG_Quaternion::From_Vector(Vector_Normalized_Accel);

    EKF_Quaternion.Init(matrix_q, matrix_r, matrix_p, vector_x);

    EKF_Quaternion.Config_Nonlinear_State_Model(EKF_Function_F, EKF_Function_Jacobian_F_X, EKF_Function_Jacobian_F_W);
    EKF_Quaternion.Config_Nonlinear_Measurement_Model(EKF_Function_H, EKF_Function_Jacobian_H_X, EKF_Function_Jacobian_H_V);

    EKF_Pre_Timestamp = dwt_bmi088->GetTimeline_us();
}
/**
 * @brief 加速计初始化
 * @return 初始化结果
 */
uint8_t T_BMI088_c::bmi088_accel_init(void)
{
    // check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    // BMI088_ACCEL_NS_L();                    \]
    // BMI088_read_write_byte((BMI088_ACC_CHIP_ID) | 0x80);   \]
    // BMI088_read_write_byte(0x55);           \]
    // (res) = BMI088_read_write_byte(0x55);  \]
    // BMI088_ACCEL_NS_H();                    \]

    dwt_bmi088->Delay_s(0.01);

    // BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte((BMI088_ACC_CHIP_ID) | 0x80);
    BMI088_read_write_byte(0x55);
    (res) = BMI088_read_write_byte(0x55);
    BMI088_ACCEL_NS_H();

    dwt_bmi088->Delay_s(0.01);
    // accel software reset
    // BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_ACCEL_NS_L();
    BMI088_write_single_reg((BMI088_ACC_SOFTRESET), (BMI088_ACC_SOFTRESET_VALUE));
    BMI088_ACCEL_NS_H();

    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    dwt_bmi088->Delay_s(0.08);
    // check commiunication is normal after reset
    // BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte((BMI088_ACC_CHIP_ID) | 0x80);
    BMI088_read_write_byte(0x55);
    (res) = BMI088_read_write_byte(0x55);
    BMI088_ACCEL_NS_H();
    dwt_bmi088->Delay_s(0.01);
    // BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, this->res);
    BMI088_ACCEL_NS_L();
    BMI088_read_write_byte((BMI088_ACC_CHIP_ID) | 0x80);
    BMI088_read_write_byte(0x55);
    (res) = BMI088_read_write_byte(0x55);
    BMI088_ACCEL_NS_H();
    dwt_bmi088->Delay_s(0.01);

    // check the "who am I"
    if (this->res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (this->write_reg_num = 0; this->write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; this->write_reg_num++)
    {

        BMI088_accel_write_single_reg(BMI088_Accel_Init_Table[this->write_reg_num][0], BMI088_Accel_Init_Table[this->write_reg_num][1]);
        dwt_bmi088->Delay_s(0.001);

        BMI088_accel_read_single_reg(BMI088_Accel_Init_Table[this->write_reg_num][0], this->res);
        dwt_bmi088->Delay_s(0.001);

        if (this->res != BMI088_Accel_Init_Table[this->write_reg_num][1])
        {
            this->error |= BMI088_Accel_Init_Table[this->write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
 * @brief 陀螺计初始化
 * @return 初始化结果
 */
uint8_t T_BMI088_c::bmi088_gyro_init(void)
{
    // check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->Delay_s(0.001);

    // reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    dwt_bmi088->Delay_s(0.08);
    // check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->Delay_s(0.001);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, this->res);
    dwt_bmi088->Delay_s(0.001);

    // check the "who am I"
    if (this->res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (this->write_reg_num = 0; this->write_reg_num < BMI088_WRITE_GYRO_REG_NUM; this->write_reg_num++)
    {

        BMI088_gyro_write_single_reg(BMI088_Gyro_Init_Table[this->write_reg_num][0], BMI088_Gyro_Init_Table[this->write_reg_num][1]);
        dwt_bmi088->Delay_s(0.001);

        BMI088_gyro_read_single_reg(BMI088_Gyro_Init_Table[this->write_reg_num][0], this->res);
        dwt_bmi088->Delay_s(0.001);

        if (this->res != BMI088_Gyro_Init_Table[this->write_reg_num][1])
        {
            this->write_reg_num--;
            this->error |= BMI088_Accel_Init_Table[this->write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}
/**
 * @brief 四元数状态转移函数
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 1> T_BMI088_c::EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 1> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];
    matrix_omega[3][3] = 0.0f;

    matrix_result = Vector_X + 0.5f * D_T * matrix_omega * Vector_X;

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 4> T_BMI088_c::EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 4> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];

    matrix_result = Namespace_ALG_Matrix::Identity<4, 4>() + 0.5f * D_T * matrix_omega;

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对过程噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 3> T_BMI088_c::EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 3> matrix_result;

    // 四元数矩阵
    Class_Matrix_f32<4, 3> matrix_q;
    matrix_q[0][0] = -Vector_X[1][0];
    matrix_q[0][1] = -Vector_X[2][0];
    matrix_q[0][2] = -Vector_X[3][0];
    matrix_q[1][0] = Vector_X[0][0];
    matrix_q[1][1] = -Vector_X[3][0];
    matrix_q[1][2] = Vector_X[2][0];
    matrix_q[2][0] = Vector_X[3][0];
    matrix_q[2][1] = Vector_X[0][0];
    matrix_q[2][2] = -Vector_X[1][0];
    matrix_q[3][0] = -Vector_X[2][0];
    matrix_q[3][1] = Vector_X[1][0];
    matrix_q[3][2] = Vector_X[0][0];

    matrix_result = 0.5f * D_T * matrix_q;

    return matrix_result;
}

/**
 * @brief 四元数测量函数
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 1> T_BMI088_c::EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 1> matrix_result;

    matrix_result[0][0] = 2.0f * (Vector_X[1][0] * Vector_X[3][0] - Vector_X[0][0] * Vector_X[2][0]);
    matrix_result[1][0] = 2.0f * (Vector_X[2][0] * Vector_X[3][0] + Vector_X[0][0] * Vector_X[1][0]);
    matrix_result[2][0] = Vector_X[0][0] * Vector_X[0][0] - Vector_X[1][0] * Vector_X[1][0] - Vector_X[2][0] * Vector_X[2][0] + Vector_X[3][0] * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 4> T_BMI088_c::EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 4> matrix_result;

    matrix_result[0][0] = -2.0f * Vector_X[2][0];
    matrix_result[0][1] = 2.0f * Vector_X[3][0];
    matrix_result[0][2] = -2.0f * Vector_X[0][0];
    matrix_result[0][3] = 2.0f * Vector_X[1][0];

    matrix_result[1][0] = 2.0f * Vector_X[1][0];
    matrix_result[1][1] = 2.0f * Vector_X[0][0];
    matrix_result[1][2] = 2.0f * Vector_X[3][0];
    matrix_result[1][3] = 2.0f * Vector_X[2][0];

    matrix_result[2][0] = 2.0f * Vector_X[0][0];
    matrix_result[2][1] = -2.0f * Vector_X[1][0];
    matrix_result[2][2] = -2.0f * Vector_X[2][0];
    matrix_result[2][3] = 2.0f * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对测量噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 */
Class_Matrix_f32<3, 3> T_BMI088_c::EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    return Namespace_ALG_Matrix::Identity<3, 3>();
}

/**
 * @brief 卡方检验
 *
 */
void T_BMI088_c::Accel_Chi_Square_Calculate()
{
    Class_Matrix_f32<3, 1> vector_error;
    Class_Matrix_f32<3, 4> matrix_h_x;
    Class_Matrix_f32<3, 3> matrix_d;

    vector_error = Vector_Normalized_Accel - EKF_Quaternion.Function_H(EKF_Quaternion.Vector_X, D_T);

    matrix_h_x = EKF_Quaternion.Function_Jacobian_H_X(EKF_Quaternion.Vector_X, D_T);

    matrix_d = matrix_h_x * EKF_Quaternion.Matrix_P_Prior * matrix_h_x.Get_Transpose() + EKF_Quaternion.Matrix_R;

    Accel_Chi_Square_Loss = (vector_error.Get_Transpose() * matrix_d.Get_Inverse() * vector_error)[0][0];
}

void T_BMI088_c::BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(this->_ACCEL_CS_GPIO_Port, this->_ACCEL_CS_Pin, GPIO_PIN_RESET);
}
void T_BMI088_c::BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(this->_ACCEL_CS_GPIO_Port, this->_ACCEL_CS_Pin, GPIO_PIN_SET);
}
void T_BMI088_c::BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(this->_GYRO_CS_GPIO_Port, this->_GYRO_CS_Pin, GPIO_PIN_RESET);
}
void T_BMI088_c::BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(this->_GYRO_CS_GPIO_Port, this->_GYRO_CS_Pin, GPIO_PIN_SET);
}

#if defined(BMI088_USE_SPI)
uint8_t T_BMI088_c::BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_WHO, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
void T_BMI088_c::BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
void T_BMI088_c::BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}
void T_BMI088_c::BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {
        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
