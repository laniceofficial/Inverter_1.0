/*************************** Dongguan-University of Technology -ACE**************************
 * @file    unitree_motor.cpp
 * @author  胡炜
 * @version V1.0
 * @date    2024/10/24
 * @brief
 ******************************************************************************
 * @verbatim
 *  宇树1a1电机驱动，MIT控制，使用rs485通信，通信速率为4.8Mbt，DMA发送，DMA空闲接收
 *  使用方法：
 *      构建初始化结构体，创建对象
 *      设置设定值，调用控制发送函数
 *  demo：
 *   A1_motor_n::A1_motor_init_t motor_init2 = {
        .motor_id = 0,
        .angle_init_PID = {
        .Kp = 2,
        .Ki = 0,
        .Kd = 0,
        .ActualValueSource = NULL,
        .mode = NONE,
        },
        .motor_usart_instance = {
        .usart_handle_ = &huart1   //这里只配置是哪个串口就行
        },
        .radius = 10.0f,
    };
    //创建对象
    A1_motor_n::A1_motor_c* a1motor = new A1_motor_n::A1_motor_c(motor_init2);
    //力矩控制模式
    a1motor->A1MotorSetMulti(0.05,0,0,0);
    while(1)
    {
        a1motor->A1MotorControl();
        text_task_dwt->ECF_DWT_Delay_ms(2);
    }
 * @attention
 * MIT还需要使用到PID吗？要根据实际使用来确定
 * @version           time
 * v1.0   基础版本（未连接电机测试）
 ************************** Dongguan-University of Technology -ACE***************************/
//TODO2：加入守护线程
#include "unitree_motor.hpp"

using namespace A1_motor_n;
using namespace Motor_General_Def_n;

//初始化
uint8_t A1_motor_c::usart_idx_ = 0;
uint8_t A1_motor_c::idx_[] = {{0},{0}};
A1_motor_c* A1_motor_c::A1motor_instance_[][Unitree_MOTOR_MX_CNT] = {{NULL,NULL,NULL},{NULL,NULL,NULL}};


const float A1motor_Kp = 0.01f;
const float A1motor_Kw = 0.5f;

#define RE_RX_BUFFER_SIZE 78u // 接收缓冲区大小
#define A1_send_DataDeal(Ax, T1, W1, Pos1, K_P1, K_W1) \
    {                                                  \
        (Ax)->T = T1 * 256;                            \
        (Ax)->W = W1 * 128;                            \
        (Ax)->Pos = Pos1 * 16384 / 3.1415926 / 2.0;    \
        (Ax)->K_P = K_P1 * 2048;                       \
        (Ax)->K_W = K_W1 * 1024;                       \
    }

//串口接收数组
uint8_t A1_rx_buf[Unitree_MOTOR_USART_MX_CNT][RE_RX_BUFFER_SIZE];

//回调函数声明
static void A1_motor_n::A1_motor_callback(uint8_t* buf,uint16_t len);

//CRC校验
uint32_t A1_motor_c::crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

float A1_motor_c::loop_restriction_float(float num, float max_num, float limit_num)
{
    if (num > limit_num)
    {
        num -= max_num;
    }
    return num;
}

/**
 * @brief 宇树A1电机初始化函数
 * @param 宇树A1电机初始化结构体
 * @note  对创建的实例进行分组（避免使用太多串口资源，最多分两组，也就是两个串口），每个串口总线最多搭载三个A1
 *        保存在A1motor_instance_指针数组中
 */
A1_motor_c::A1_motor_c(A1_motor_init_t a1_init):
    motor_id(a1_init.motor_id), 
    angle_PID(a1_init.angle_init_PID), 
    speed_PID(a1_init.speed_init_PID),
    pid_ref_(0),
    radius_(a1_init.radius),
    reduction_radio_(9.1f)
{
    int8_t usart_temp_idx = -1;
    //判断该串口是否注册
    for (int8_t i = 0; i < usart_idx_; ++i)
    {
        if (a1_init.motor_usart_instance.usart_handle_ == A1motor_instance_[i][0]->motor_usart_instance.usart_handle)
        {
            usart_temp_idx = i;
            this->motor_usart_instance = A1motor_instance_[i][0]->motor_usart_instance;
            break;
        }
    }
    
    if (usart_temp_idx == -1)
    {//未注册
        usart_temp_idx = usart_idx_;
        usart_idx_++;
        if(usart_idx_ > Unitree_MOTOR_USART_MX_CNT) //超过串口总数限制
            return;
        a1_init.motor_usart_instance.rx_type_ = USART_N::USART_RX_DMA_IDLE;
        a1_init.motor_usart_instance.tx_type_ = USART_N::USART_TX_DMA;
        a1_init.motor_usart_instance.usart_rx_callback_ptr_ = A1_motor_n::A1_motor_callback;//回调函数
        a1_init.motor_usart_instance.rx_size_ = RE_RX_BUFFER_SIZE;
        a1_init.motor_usart_instance.rx_buff_ptr = A1_rx_buf[usart_temp_idx];//接收数组
        this->motor_usart_instance.USART_init(a1_init.motor_usart_instance);
    }

    this->send.send_data.head[0] = 0xFE;
    this->send.send_data.head[1] = 0xEE;
    this->send.send_data.motorID = this->motor_id;
    this->send.send_data.reserved1 = 0x00;
    this->send.send_data.mode = 0;
    this->send.send_data.ModifyBit = 0xFF;
    this->send.send_data.ReadBit = 0x00;
    this->send.send_data.reserved1 = 0x00;
    this->send.send_data.Modify.u32 = 0x00;
    this->send.send_data.LowHzMotorCmdIndex = 0x00;
    this->send.send_data.reserved3.u32 = 0x00;

    this->motor_settings.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    this->control_type = ZEROTORQUE;
    A1MotorEnable();
    A1MotorSetKP(A1motor_Kp);
    A1MotorSetKW(A1motor_Kw);
    memset(&(this->measure), 0, sizeof(this->measure));
    A1motor_instance_[usart_temp_idx][idx_[usart_temp_idx]++] = this;
}


//TODO1:是否需要pid？
void A1_motor_c::A1MotorControl()
{

    // 为防止串口堵塞，每个总线上只有一个电机发送控制，对每个电机实际发送频率为任务频率/总线上电机数
    if (this->control_type == MULTI)
    {
        this->control.W = this->angle_PID.ECF_PID_Calculate(this->control.Pos);
    }
       
    //电机反转
    if (this->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        //motor->control.Pos *= -1;
        this->control.W *= -1;
        this->control.T *= -1;
    }
    // 发送
    //根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des即可实现匀速转动;kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出。
    //注意：对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控。
    switch (this->control_type)
    {
    case TORQUE:
        A1_send_DataDeal(&this->send.send_data, this->control.T, 0, 0, 0, 0);
        break;
    case POISITION:
        A1_send_DataDeal(&this->send.send_data, 0, 0, this->control.Pos, this->control.K_P, this->control.K_W);
        break;
    case SPEED:
        A1_send_DataDeal(&this->send.send_data, 0, this->control.W, 0, 0, this->control.K_W);
        break;
    case MULTI:
        A1_send_DataDeal(&this->send.send_data, this->control.T, this->control.W, this->control.Pos, this->control.K_P, this->control.K_W);
        break;
    default:
        A1_send_DataDeal(&this->send.send_data, 0, 0, 0, 0, 0); // 无力
        break;
    }
        
    this->send.send_data.crc.u32 = crc32_core(((uint32_t *)&(this->send.send_array)), 7);
    // 若该电机处于停止状态,直接将模式置零，否则设置为10。
    if (MOTOR_STOP == this->stop_flag)
        this->send.send_data.mode = 0;
    else
        this->send.send_data.mode = 10;

        // int cnt = motor->motor_usart_instance->usart_handle->RxXferCount;
        // int last_cnt = cnt - 1;
        // while ((cnt - last_cnt) != 0)
        // {
        //     last_cnt = cnt;
        //     cnt = motor->motor_usart_instance->usart_handle->RxXferCount;
        // }
        // HAL_UART_Transmit_DMA(motor->motor_usart_instance->usart_handle, motor->send.send_array, 34);
    this->motor_usart_instance.USART_send(this->send.send_array, 34, 100);
}


/**
 * @brief A1电机数据接收解析回调函数
 * @note  统一只使用这个回调函数，会自动查找所对应的电机
 */
static void A1_motor_n::A1_motor_callback(uint8_t* buf,uint16_t len)
{
    A1_motor_c *motor;
    uint32_t CRC32 = 0xFFFFFFFF;
    int8_t usart_num = -1;
    int8_t rxidx = -1;
    for (int8_t i = 0; i < A1_motor_c::usart_idx_; ++i)//查找是属于哪个电机的信号
    {
        if ((A1_motor_c::A1motor_instance_[i][0]->motor_usart_instance.rx_buff_ptr[0] == 0xFE) && (((A1_Tx_Data_t *)A1_motor_c::A1motor_instance_[i][0]->motor_usart_instance.rx_buff_ptr)->crc.u32 != 0))//TODO：反馈强转化为发送格式？
        {
            int8_t id = A1_motor_c::A1motor_instance_[i][0]->motor_usart_instance.rx_buff_ptr[2];
            usart_num = i;
            for (int8_t j = 0; j < A1_motor_c::idx_[usart_num]; ++j)
            {
                if (A1_motor_c::A1motor_instance_[usart_num][j]->motor_id == id)
                {
                    rxidx = j;
                    break;
                }
            }
            break;
        }
    }
    if ((rxidx == -1) || (usart_num == -1)) // 对应串口不存在或电机不存在
    {
        return;
    }

    motor = A1_motor_c::A1motor_instance_[usart_num][rxidx];
    CRC32 = A1_motor_c::crc32_core((uint32_t *)buf, 18);
    memcpy((uint8_t *)&motor->measure.crc.u32, &buf[74], 4);
    if (CRC32 == motor->measure.crc.u32) //CRC校验通过
    {
        motor->measure.motor_id = buf[2];
        motor->measure.mode = buf[4];
        motor->measure.Temp = buf[6];
        motor->measure.MError = buf[7];
        motor->measure.T = (1 - T_SMOOTH_COEF) * motor->measure.T + A1_motor_c::loop_restriction_float((buf[12] | buf[13] << 8) / 256.0f, 256.0f, 128.0f) * T_SMOOTH_COEF;
        motor->measure.W = (1 - W_SMOOTH_COEF) * motor->measure.W + A1_motor_c::loop_restriction_float((buf[14] | buf[15] << 8) / 128.0f, 512.0f, 256.0f) * W_SMOOTH_COEF;
        *(uint8_t *)&motor->measure.LW = buf[16];
        *((uint8_t *)&motor->measure.LW + 1) = buf[17];
        *((uint8_t *)&motor->measure.LW + 2) = buf[18];
        *((uint8_t *)&motor->measure.LW + 3) = buf[19];
        motor->measure.Acc = A1_motor_c::loop_restriction_float((buf[26] | buf[27] << 8), 65535, 65535 / 2);
        motor->measure.Pos = A1_motor_c::loop_restriction_float((buf[30] | buf[31] << 8 | buf[32] << 16 | buf[33] << 24) / (16384.0f / PI / 2.0f), 823549 * 2, 823549);
        // motor->measure.gyro[0] = (buf[38] | buf[39] << 8) / (2000.0f * 2.0f * PI / (float)(1 << 15) / 360.0f);
        memset(buf, 0, RE_RX_BUFFER_SIZE);
        return;
    }

}

void A1_motor_c::A1MotorDataClear(void)
{
    memset(&(this->measure), 0, sizeof(this->measure));
    this->angle_PID.ECF_PID_CLEAR();
    this->speed_PID.ECF_PID_CLEAR();
}

//电机失能
void A1_motor_c::A1MotorStop()
{
    this->stop_flag = MOTOR_STOP;//+
    this->A1MotorDataClear();
}

//电机使能
void A1_motor_c::A1MotorEnable()
{
    this->stop_flag = MOTOR_ENALBED;
}

//设置电机前馈力矩
void A1_motor_c::A1MotorSetT(float T)
{
    this->control.T = T;
}

//设置电机速度命令
void A1_motor_c::A1MotorSetW(float W)
{
    this->control.W = W;
}

//设置电机位置刚度
void A1_motor_c::A1MotorSetKP(float KP)
{
    this->control.K_P = KP;
}

//设置电机速度刚度
void A1_motor_c::A1MotorSetKW(float KW)
{
    this->control.K_W = KW;
}

//设置电机工作模式
void A1_motor_c::A1MotorSetControlType(Control_Type_e ControlType)
{
    this->control_type = ControlType;
}

//设置电机控制参数
void A1_motor_c::A1MotorSetMulti(float T, float Pos, float KP, float KW)
{
    this->control_type = MULTI;
    this->control.T = T;
    // motor->control.W = W;W用pid单独控制
    this->control.K_P = KP;
    this->control.K_W = KW;
    this->control.Pos = Pos;
}


