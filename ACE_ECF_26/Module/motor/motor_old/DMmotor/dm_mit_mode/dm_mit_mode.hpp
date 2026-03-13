#ifndef __DM_MIT_MODE_HPP
#define __DM_MIT_MODE_HPP

#include "dm_motor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef __cplusplus
}
#endif

namespace DM_Motor_n
{
    //目前只有适应H7FDCAN的类，可以像我这拓展(很方便😋)有用到再说
    class DM_Mit_Mode_c:public DM_Motor_Main_c,public BSP_CAN_Part_n::CANInstance_c
    {     
        private:
        void TxDataProcessing(float _pos,float _vel,float _Kp,float _Kd,float _torq);
        MIT_HandOver_e mode = POSITION;
        public:
        DM_Mit_Mode_c(
                      DM_ModePrame_s *param_
                      );
        void HandOver(float _pos,float _vel,float _Kp,float _Kd,float _torq);
        void Transmit(float _pos,float _vel,float _Kp,float _Kd,float _torq);
        void StateSet(DM_StartState_e State);

        //用户层操作
        void Postion();
        void Torque();
        void Speed();
    };
}






#endif /*__DM_MIT_MODE_HPP*/