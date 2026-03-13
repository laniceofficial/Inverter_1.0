#ifndef BMI088
#define BMI088

#include "bsp_spi.hpp"
#include "BMI088reg_test.hpp"
#include "bsp_dwt.hpp"
#include "alg_quaternion.hpp"
#include "alg_filter_ekf.hpp"
// extern T2_BMI088_c *T2_BMI088_c::BMI_List[1] = {nullptr};
static BSP_n::DWT_c *dwt_bmi088 = BSP_n::DWT_c::Get_DwtInstance();
const  uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
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
const uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {

        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, // 改动会导致计算延缓
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
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
class T2_BMI088_c
{

public:
    inline uint8_t Init(SPI_HandleTypeDef *bmi088_SPI, uint8_t calibrate,
                        GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                        uint16_t _ACCEL_CS_Pin,
                        GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                        uint16_t _GYRO_CS_Pin,
                        BSP_n::SPI_c::TransType_t spi_work_mode);
    static void AccCallBack(BSP_n::SPI_c *register_instance);
    static void GyroCallBack(BSP_n::SPI_c *register_instance);
    void AccHandle();
    void GyroHandle();
    void Calibrate_MPU_Offset();
    void BMI_UpDate(void);
    void BMI088_Read();
    void Attitude_Calc();
    void EXTI_Flag_Callback(uint16_t GPIO_Pin);
    IMU_Data_t RawDate; // 六轴原始数据    
    static T2_BMI088_c *BMI_List[1];
protected:
    // 数据准备好标志
    bool Accel_Ready_Flag = false;
    uint64_t Accel_Ready_Timestamp = 0;
    bool Gyro_Ready_Flag = false;
    uint64_t Gyro_Ready_Timestamp = 0;
    bool Temperature_Ready_Flag = false;
    // 数据传输标志
    bool Accel_Transfering_Flag = false;
    uint64_t Accel_Transfering_Timestamp = 0;
    bool Gyro_Transfering_Flag = false;
    uint64_t Gyro_Transfering_Timestamp = 0;
    bool Temperature_Transfering_Flag = false;
    uint64_t Temperature_Transfering_Timestamp = 0;
    // 数据更新标志
    bool Accel_Update_Flag = false;
    uint64_t Accel_Update_Timestamp = 0;
    bool Gyro_Update_Flag = false;
    uint64_t Gyro_Update_Timestamp = 0;
    // 数据合法标志
    bool Accel_Valid_Flag = false;
    bool Gyro_Valid_Flag = false;

    BSP_n::SPI_c::TransType_t work_mode;

    uint8_t add_flag = 0;
    BSP_n::SPI_c BMI088_Accel;
    BSP_n::SPI_c BMI088_Gyro;
    uint8_t res = 0;
    uint8_t error = BMI088_NO_ERROR;
    uint8_t caliOffset = 1; // 是否开启较准零飘
    // 用于获取两次采样之间的时间间隔
    float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
    float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
     uint8_t gyrobuf[8] = {0};
     uint8_t accbuf[8] = {0};
    /********函数部分********* */
    inline uint8_t InitAcc(SPI_HandleTypeDef *bmi088_SPI,
                           GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                           uint16_t _ACCEL_CS_Pin,
                           BSP_n::SPI_c::TransType_t spi_work_mode);
    inline uint8_t InitGyro(SPI_HandleTypeDef *bmi088_SPI,
                            GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                            uint16_t _GYRO_CS_Pin,
                            BSP_n::SPI_c::TransType_t spi_work_mode);

    inline void ReadSingleRegister(const uint8_t reg, uint8_t *data);
    inline uint8_t ReadWriteByte(uint8_t tx_data);
    inline void WriteSingleRegister(const uint8_t reg, uint8_t data);
    inline void ReadMultipleRegister(const uint8_t reg, uint8_t *data, uint8_t len);
    inline void WriteMultipleRegister(const uint8_t reg, uint8_t *data, uint8_t len);
    void ReadInBlock();
    void ReadInDMA();
};

inline uint8_t T2_BMI088_c::InitAcc(SPI_HandleTypeDef *bmi088_SPI,
                                 GPIO_TypeDef *_ACCEL_CS_GPIO_Port,
                                 uint16_t _ACCEL_CS_Pin,
                                 BSP_n::SPI_c::TransType_t spi_work_mode)
{
    BMI088_Accel.Init(bmi088_SPI, _ACCEL_CS_GPIO_Port, _ACCEL_CS_Pin,
                      spi_work_mode,
                      AccCallBack);

    ReadSingleRegister(BMI088_ACC_CHIP_ID, &res); // 切换到spi模式
    dwt_bmi088->Delay_ms(10);
    ReadSingleRegister(BMI088_ACC_CHIP_ID, &res);
    dwt_bmi088->Delay_ms(10);
    WriteSingleRegister(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    dwt_bmi088->Delay_ms(80);
    // check commiunication is normal after reset
    ReadSingleRegister(BMI088_ACC_CHIP_ID, &res);
    dwt_bmi088->Delay_ms(10);
    ReadSingleRegister(BMI088_ACC_CHIP_ID, &res);
    dwt_bmi088->Delay_ms(10);
    // check the "who am I"
    if (this->res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }
    uint8_t write_reg_num = 0;
    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        WriteSingleRegister(BMI088_Accel_Init_Table[write_reg_num][0], BMI088_Accel_Init_Table[write_reg_num][1]);
        dwt_bmi088->Delay_ms(1);

        ReadSingleRegister(BMI088_Accel_Init_Table[write_reg_num][0], &res);
        dwt_bmi088->Delay_ms(1);

        if (this->res != BMI088_Accel_Init_Table[write_reg_num][1])
        {
            this->error |= BMI088_Accel_Init_Table[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
inline uint8_t T2_BMI088_c::InitGyro(SPI_HandleTypeDef *bmi088_SPI,
                                  GPIO_TypeDef *_GYRO_CS_GPIO_Port,
                                  uint16_t _GYRO_CS_Pin,
                                  BSP_n::SPI_c::TransType_t spi_work_mode)
{
    BMI088_Gyro.Init(bmi088_SPI, _GYRO_CS_GPIO_Port, _GYRO_CS_Pin,
                     spi_work_mode,
                     GyroCallBack);
    ReadSingleRegister(BMI088_GYRO_CHIP_ID, &res); // 切换到spi模式
    dwt_bmi088->Delay_ms(1);
    ReadSingleRegister(BMI088_GYRO_CHIP_ID, &res);
    dwt_bmi088->Delay_ms(1);
    // reset the gyro sensor
    WriteSingleRegister(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    // HAL_Delay(BMI088_LONG_DELAY_TIME);
    dwt_bmi088->Delay_ms(80);
    // check commiunication is normal after reset
    ReadSingleRegister(BMI088_GYRO_CHIP_ID, &this->res);
    dwt_bmi088->Delay_ms(1);
    ReadSingleRegister(BMI088_GYRO_CHIP_ID, &this->res);
    dwt_bmi088->Delay_ms(1); // check the "who am I"
    if (this->res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }
    // set gyro sonsor config and check
    uint8_t write_reg_num = 0;
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        WriteSingleRegister(BMI088_Gyro_Init_Table[write_reg_num][0], BMI088_Gyro_Init_Table[write_reg_num][1]);
        dwt_bmi088->Delay_ms(1);

        ReadSingleRegister(BMI088_Gyro_Init_Table[write_reg_num][0], &res);
        dwt_bmi088->Delay_ms(1);

        if (this->res != BMI088_Gyro_Init_Table[write_reg_num][1])
        {
            this->error |= BMI088_Gyro_Init_Table[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
inline void T2_BMI088_c::ReadSingleRegister(const uint8_t reg, uint8_t *data)
{
    // BSP_n::SPI_c::ReadSingleRegister(BMI088_GYRO_IIC_ADDRESSE, BMI088_GYRO_CHIP_ID_VALUE);

    ReadWriteByte(reg | BMI088_READ_MASK);
    ReadWriteByte(0x55);         // 为什么需要读两遍
    *data = ReadWriteByte(0x55); // 0x55为保留位
    uint8_t tx_buf[10] = {};
    
}
inline uint8_t T2_BMI088_c::ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data;
    BMI088_Accel.TransRecv(&rx_data, &tx_data, 1);
    return rx_data;
}
inline void T2_BMI088_c::WriteSingleRegister(const uint8_t reg, uint8_t data)
{
    ReadWriteByte(reg);
    ReadWriteByte(data);
}
inline void T2_BMI088_c::ReadMultipleRegister(const uint8_t reg, uint8_t *data, uint8_t len)
{
    ReadWriteByte(reg | BMI088_READ_MASK);
    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = ReadWriteByte(0x55); // 0x55为保留位
    }
}
inline void T2_BMI088_c::WriteMultipleRegister(const uint8_t reg, uint8_t *data, uint8_t len)
{
}
#endif // !BMI088
