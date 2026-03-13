/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dm_ls_mode.cpp
 * @author  KazuHa12441
 * @version V1.0
 * @date    2024/10/2
 * @brief   达妙电机mit模式
 * 
 * @todo: 小凳有需要的话来完善吧CAN吧
 *        目前代码有写赘余
********************************************************************************************
 * @attention 
 * 1.创建一个参数结构体DM_ModePrame_s
 * 2.创建一个对象dm_ls_mode_c 
 * 3.调用SetCallBack函数将回调函数写入对象中
 * 4.调用StartStateSet进行初始状态选择
 * 5.调用对象的方法Transimit发送电机数据()
 * 6.接收的数据会在类变量get_data_中
 * 7.如果想调用内部数据 可以调用DM_GetMotorData_s *ReturnMotorData(void)，它将返回一个数据指针
**********************************************************************************************/
#include "dm_ls_mode.hpp"

namespace DM_Motor_n
{

    DM_LS_Mode_c::DM_LS_Mode_c(FDCAN_HandleTypeDef *fdcan_handle,
                               DM_ModePrame_s *param_,
                               uint32_t tx_id,
                               uint32_t rx_id,
                               uint32_t SAND_IDE)
        : DM_Motor_Main_c(param_),
          FDCANInstance_c(fdcan_handle,tx_id + (uint32_t)0x100,rx_id,SAND_IDE)
    {
        get_data_.nowState = UNABLE;
        mode_              = LOCTION_SPEED;
        dm_idx_++;
    }
    
    /// @brief 位置速度模式发送解码
    /// @param _pos 位置
    /// @param _vel 速度
    void DM_LS_Mode_c::TxDataProcessing(float _pos, float _vel)
    {
        uint8_t *pbuf, *vbuf;
        pbuf = (uint8_t *)&_pos;
        vbuf = (uint8_t *)&_vel;

        txbuffer_[0] = *pbuf;
        txbuffer_[1] = *(pbuf + 1);
        txbuffer_[2] = *(pbuf + 2);
        txbuffer_[3] = *(pbuf + 3);
        txbuffer_[4] = *vbuf;
        txbuffer_[5] = *(vbuf + 1);
        txbuffer_[6] = *(vbuf + 2);
        txbuffer_[7] = *(pbuf + 3);
    }

    /// @brief 状态设置 
    /// @param State 
    void DM_LS_Mode_c::StateSet(DM_StartState_e State)
    {
        DMMotorStateSet(State);
        TxDataToBuffer(tx_buff, txbuffer_);
        ECF_Transmit(TXTIME);
    }

    void DM_LS_Mode_c::Transmit(float _pos, float _vel)
    {
        TxDataProcessing(_pos, _vel);
        TxDataToBuffer(tx_buff, txbuffer_);
        ECF_Transmit(TXTIME);
    }
}