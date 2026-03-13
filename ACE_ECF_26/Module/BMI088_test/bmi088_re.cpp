#include "bmi088_re.hpp"
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

#endif
T2_BMI088_c *T2_BMI088_c::BMI_List[1] = {nullptr};

inline uint8_t T2_BMI088_c::Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate,
                              GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                              uint16_t _ACCEL_CS_Pin,
                              GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                              uint16_t _GYRO_CS_Pin,
                              BSP_n::SPI_c::TransType_t spi_work_mode)
{

    // 强制复位两条片选引脚，防止笨b配错cubemx的片选引脚高低状态
    HAL_GPIO_WritePin(_ACCEL_CS_GPIO_Port, _ACCEL_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_GYRO_CS_GPIO_Port, _GYRO_CS_Pin, GPIO_PIN_RESET);
    if (add_flag == 0)
    {
        add_flag = 1;
        uint8_t BMI_List_Idx = 0;
        while (BMI_List[BMI_List_Idx] != nullptr)
            BMI_List_Idx++; // 越界风险
        BMI_List[BMI_List_Idx] = this;
    }
    do
    {
        this->error = BMI088_NO_ERROR;
        // 此处先用阻塞模式初始化
        this->error |= InitAcc(bmi088_SPI, _ACCEL_CS_GPIO_Port, _ACCEL_CS_Pin, BSP_n::SPI_c::TransType_t::BLOCK);
        this->error |= InitGyro(bmi088_SPI, _GYRO_CS_GPIO_Port, _GYRO_CS_Pin, BSP_n::SPI_c::TransType_t::BLOCK);

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
    work_mode = spi_work_mode;
    BMI088_Accel.SetMode(spi_work_mode);
    BMI088_Gyro.SetMode(spi_work_mode);
    return this->error;
}
void T2_BMI088_c::Calibrate_MPU_Offset()
{
    static float startTime;
    static uint16_t CaliTimes = 6000; // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp = 0.0f, gNormMax = 0.0f, gNormMin = 0.0f;
    int16_t caliCount = 0; // 调试用的变量
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
            ReadMultipleRegister(BMI088_ACCEL_XOUT_L, buf, 6);
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

            ReadMultipleRegister(BMI088_GYRO_CHIP_ID, buf, 8);
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
        ReadMultipleRegister(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        RawDate.TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
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

/**
 * @brief 所有陀螺仪数值、姿态更新
 */
void T2_BMI088_c::BMI_UpDate(void)
{
    uint8_t BMI_List_Idx = 0;
    while (BMI_List_Idx < (sizeof(BMI_List) / sizeof(BMI_List[0])) && BMI_List[BMI_List_Idx] != nullptr)
    {
        float in = dwt_bmi088->GetTimeline_ms();
        BMI_List[BMI_List_Idx]->BMI088_Read();
        BMI_List[BMI_List_Idx]->Attitude_Calc();
        BMI_List_Idx++;
        float out = dwt_bmi088->GetTimeline_ms();
        float delta_t = out - in; // 0.876ms
    }
}
void T2_BMI088_c::BMI088_Read()
{
    switch (work_mode)
    {
    case BSP_n::SPI_c::TransType_t::DMA:
        ReadInDMA();
        break;
    case BSP_n::SPI_c::TransType_t::BLOCK:
        ReadInBlock();
        break;
    default:
        break;
    }
}

// void BMI088_RecCallBack(BSP_n::SPI_c *register_instance)
// {

// }
void T2_BMI088_c::ReadInDMA()
{
    if (Accel_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
    {
        Accel_Transfering_Flag = true;
        ReadMultipleRegister(BMI088_ACCEL_XOUT_L, accbuf, 6);
        Accel_Ready_Flag = false;
        return;
    }
    else if (Gyro_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
    {
        Gyro_Transfering_Flag = true;
        ReadMultipleRegister(BMI088_GYRO_CHIP_ID, gyrobuf, 8);
        Gyro_Ready_Flag = false;
        return;
    }
    else if (Temperature_Ready_Flag && !Accel_Transfering_Flag && !Gyro_Transfering_Flag && !Temperature_Transfering_Flag)
    {
        Temperature_Transfering_Flag = true;
        ReadMultipleRegister(BMI088_TEMP_M, accbuf, 2);
        Temperature_Transfering_Flag = false;
        return;
    }
}
void T2_BMI088_c::ReadInBlock()
{
    static uint8_t buf[8] = {0};
    static int16_t bmi088_raw_temp;
    // 轴加速度计原始数据读取
    ReadMultipleRegister(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    RawDate.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    RawDate.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    RawDate.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
    // 角加速度计原始数据读取
    // Accel_Update_Flag = true;
    ReadMultipleRegister(BMI088_GYRO_CHIP_ID, buf, 8);
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
    ReadMultipleRegister(BMI088_TEMP_M, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    // Vector_Original_Accel[0][0] = RawDate.Accel[0];
    // Vector_Original_Accel[1][0] = RawDate.Accel[1];
    // Vector_Original_Accel[2][0] = RawDate.Accel[2];
    // Vector_Original_Gyro[0][0] = RawDate.Gyro[0];
    // Vector_Original_Gyro[1][0] = RawDate.Gyro[1];
    // Vector_Original_Gyro[2][0] = RawDate.Gyro[2];
    RawDate.Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    // 防止NaN流入算法, 加速度计数据不合法直接丢弃, 陀螺仪数据不合法则使用上次数据

    // Vector_Normalized_Accel = Vector_Original_Accel.Get_Normalization();
    // Accel_Valid_Flag = true;
    // if (alg_n::IsInvalid_loat(Vector_Original_Accel[0][0]) || alg_n::IsInvalid_loat(Vector_Original_Accel[1][0]) || alg_n::IsInvalid_loat(Vector_Original_Accel[2][0]))
    // {
    //     Accel_Valid_Flag = false;
    // }
    // Gyro_Valid_Flag = true;
    // if (alg_n::IsInvalid_loat(Vector_Original_Gyro[0][0]) || alg_n::IsInvalid_loat(Vector_Original_Gyro[1][0]) || alg_n::IsInvalid_loat(Vector_Original_Gyro[2][0]))
    // {
    //     Gyro_Valid_Flag = false;
    //     Vector_Original_Gyro = Vector_Pre_Original_Gyro;
    // }
}
void T2_BMI088_c::Attitude_Calc()
{
    
}
/**
 * @brief EXTI中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void T2_BMI088_c::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_Pin)
    {
        Accel_Ready_Flag = true;
    }
    else if (GPIO_Pin == INT2_Pin)
    {
        Gyro_Ready_Flag = true;
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if (!init_finished)
    // {
    //     return;
    // }
    if (GPIO_Pin == INT1_Pin || GPIO_Pin == INT2_Pin)
    {
        T2_BMI088_c::BMI_List[0]->EXTI_Flag_Callback(GPIO_Pin);
    }
}
void T2_BMI088_c::AccCallBack(BSP_n::SPI_c *register_instance)
{
    for (int i = 0; i < 1; i++)
    {
        if (BMI_List[i] && (&(BMI_List[i]->BMI088_Accel) == register_instance))
        {
            BMI_List[i]->AccHandle();
            break;
        }
    }
}
void T2_BMI088_c::GyroCallBack(BSP_n::SPI_c *register_instance)
{
    for (int i = 0; i < 1; i++)
    {
        if (BMI_List[i] && (&(BMI_List[i]->BMI088_Gyro) == register_instance))
        {
            BMI_List[i]->GyroHandle();

            break;
        }
    }
}
void T2_BMI088_c::AccHandle()
{
    static int16_t bmi088_raw_temp;
    if (accbuf[0] == (BMI088_ACCEL_XOUT_L | BMI088_READ_MASK))
    {
        bmi088_raw_temp = (int16_t)((accbuf[1]) << 8) | accbuf[0];
        RawDate.Accel[0] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
        bmi088_raw_temp = (int16_t)((accbuf[3]) << 8) | accbuf[2];
        RawDate.Accel[1] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
        bmi088_raw_temp = (int16_t)((accbuf[5]) << 8) | accbuf[4];
        RawDate.Accel[2] = bmi088_raw_temp * this->BMI088_ACCEL_SEN * RawDate.AccelScale;
        Accel_Update_Flag = true;
    }
    if (accbuf[0] == (BMI088_TEMP_M | BMI088_READ_MASK))
    {
        bmi088_raw_temp = (int16_t)((accbuf[0] << 3) | (accbuf[1] >> 5));
        if (bmi088_raw_temp > 1023)
        {
            bmi088_raw_temp -= 2048;
        }
        Temperature_Ready_Flag = true;
    }

}
void T2_BMI088_c::GyroHandle()
{
    static int16_t bmi088_raw_temp;
    if (gyrobuf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((gyrobuf[3]) << 8) | gyrobuf[2];
            RawDate.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[0];
            bmi088_raw_temp = (int16_t)((gyrobuf[5]) << 8) | gyrobuf[4];
            RawDate.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[1];
            bmi088_raw_temp = (int16_t)((gyrobuf[7]) << 8) | gyrobuf[6];
            RawDate.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN - RawDate.GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((gyrobuf[3]) << 8) | gyrobuf[2];
            RawDate.Gyro[0] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((gyrobuf[5]) << 8) | gyrobuf[4];
            RawDate.Gyro[1] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((gyrobuf[7]) << 8) | gyrobuf[6];
            RawDate.Gyro[2] = bmi088_raw_temp * this->BMI088_GYRO_SEN;
        }
    }
}