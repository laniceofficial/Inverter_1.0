/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dm_speed_mode.cpp
 * @author  KazuHa12441
 * @version V1.0
 * @date    2024/10/2
 * @brief   达妙电机mit模式
 * 
 * @todo: 小凳有需要的话来完善吧CAN吧
********************************************************************************************
 * @attention 
 * 1.创建一个参数结构体DM_ModePrame_s
 * 2.创建一个对象dm_speed_mode_c 
 * 3.调用SetCallBack函数将回调函数写入对象中
 * 4.调用StartStateSet进行初始状态选择
 * 5.调用对象的方法Transimit发送电机数据()
 * 6.接收的数据会在类变量get_data_中
 * 7.如果想调用内部数据 可以调用DM_GetMotorData_s *ReturnMotorData(void)，它将返回一个数据指针
**********************************************************************************************/
#include "dm_speed_mode.hpp"

namespace DM_Motor_n
{
    DM_Speed_Mode_c::DM_Speed_Mode_c(FDCAN_HandleTypeDef *fdcan_handle,
                                     DM_ModePrame_s *param_,
                                     uint32_t tx_id,
                                     uint32_t rx_id,
                                     uint32_t SAND_IDE)
        : DM_Motor_Main_c(param_),
          FDCANInstance_c(fdcan_handle, tx_id + (uint32_t)0x200, rx_id, SAND_IDE)
    {
        get_data_.nowState = UNABLE;
        mode_              = SPEED;
        dm_idx_++;
    }
    
    void DM_Speed_Mode_c::TxDataProcessing(float _vel)
    {
        uint8_t *vbuf;
        vbuf = (uint8_t *)&_vel;

        txbuffer_[0] = *vbuf;
        txbuffer_[1] = *(vbuf+1);
        txbuffer_[2] = *(vbuf+2);
        txbuffer_[3] = *(vbuf+3);
    }


     void DM_Speed_Mode_c::StateSet(DM_StartState_e State)
    {
        DMMotorStateSet(State);
        TxDataToBuffer(tx_buff, txbuffer_);
        ECF_Transmit(TXTIME);
    }

    void DM_Speed_Mode_c::Transmit(float _vel)
    {
        TxDataProcessing(_vel);
        TxDataToBuffer(tx_buff, txbuffer_);
        ECF_Transmit(TXTIME);
    }
}