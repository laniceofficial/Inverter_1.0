/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dm_motor.cpp
 * @author  KazuHa12441
 * @version V1.1
 * @date    2024/10/2
 * @brief   达妙电机父类（继承）
 * 
 * @todo: 小凳有需要的话来完善吧CAN吧
/ ********************************************************************************************/
#include "dm_motor.hpp"
 
namespace DM_Motor_n
{
    uint8_t DM_Motor_Main_c::dm_idx_ = 0;

    /// @brief DM电机父类构造，参数赋值
    /// @param param_ 参数结构体指针
    /// @param tx_id 发送id
    /// @param rx_id 接收id
    DM_Motor_Main_c:: DM_Motor_Main_c(DM_ModePrame_s *param_)
    {
        this->param_ = param_;       
    }
    
    /// @brief DM电机状态设置
    /// @param state 初始化状态枚举 
    void DM_Motor_Main_c::DMMotorStateSet(DM_StartState_e state)
    {
        for (int i = 0; i < 8; i++)
            txbuffer_[i] = 0xFF;

        switch (state) {
            case DM_UNABLE: {
                txbuffer_[7] =  0xFD;
                break;
            }
            case DM_ENABLE:
            {
                txbuffer_[7] =  0xFC;
                break;
            }
            case DM_PORTECT_ZERO_POSITION:
            {
                txbuffer_[7] =  0xFE;
                break;
            }
            case DM_CLEAR_FAULT:    
            {
                txbuffer_[7] =  0xFB;
                break;
            }
            default:
            while(1)
            {       
                //输入错误
            }
            break;
        }
    }
    
    // void DM_Motor_Main_c::RxDataDecode(BSP_CAN_Part_n::FDCANInstance_c * register_instance)
    // {
    //   int p_int, v_int, t_int;
    //   p_int=(register_instance->rx_buff[1]<<8)|register_instance->rx_buff[2];
    //   v_int=(register_instance->rx_buff[3]<<4)|(register_instance->rx_buff[4]>>4);
    //   t_int=((register_instance->rx_buff[4]&0xF)<<8)|register_instance->rx_buff[5];
    // //  get_data_.postion = uint_to_float(p_int,param_->p_min, param_->p_max, 16);// (-12.5,12.5)
    // //  get_data_.velocity = uint_to_float(v_int, param_->v_min, param_->v_max, 12);// (-45.0,45.0)
    // //  get_data_.toeque = uint_to_float(t_int,param_->t_min, param_->t_max, 12); //(-18.0,18.0)
    // // }

    /// @brief 获取数据公用接口
    /// @param  无
    DM_GetMotorData_s *DM_Motor_Main_c::ReturnMotorData(void)
    {
        return &this->get_data_;        
    }

    /// @brief float转int
    /// @param x_float 浮点数
    /// @param x_min 最小值
    /// @param x_max 最大值
    /// @param bits  bit位
    /// @return 返回处理后的数据
    uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits)
    {
        float span   = x_max - x_min;
        float offset = x_min;
        return (uint16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
    }

    /// @brief int转float
    /// @param x_int 整型
    /// @param x_min 最小值
    /// @param x_max 最大值
    /// @param bits  bit位
    /// @return 返回处理后的数据
    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        float span   = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    void TxDataToBuffer(uint8_t *pdata1,uint8_t *pdata2)
    {
        for(int i = 0;i<8;i++)
        {
            pdata1[i] = pdata2[i];
        }
    }
}