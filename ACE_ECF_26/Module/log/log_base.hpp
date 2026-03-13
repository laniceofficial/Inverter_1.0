/************************** Dongguan-University of Technology -ACE**************************
 * @file  log_base.hpp
 * @brief 日志基类
 * @author L_Zero
 * @version v26.0.0.0
 * @note
 *  *  日志基类，希望兼容本地Flash，并且也支持串口、SPI等协议下的外部Flash
 *
************************** Dongguan-University of Technology -ACE***************************/

#ifndef __LOG_BASE_DEF_HPP
#define __LOG_BASE_DEF_HPP

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

#ifdef __cplusplus
};
#endif

namespace Log_n
{
    /**
     * @brief 日志存储类型
     */
    typedef enum : uint8_t
    {
        NoType     = 0x00u,
        LocalFlash = 0x01u,
        // 后续可以添加...
    } LogSaveType_e;

    /**
     * @brief 信息类型
     */
    typedef enum class MessageType_e : uint8_t
    {
        Notice   = 0x00u,
        Warning  = 0x01u,
        Error    = 0x02u,
        Fatal    = 0x03u,
        UserDef1 = 0x04u,
        UserDef2 = 0x05u,
        UserDef3 = 0x06u,
        UserDef4 = 0x07u,
    } MessageType_e;

    /**
     * @brief 工作状态
     */
    typedef enum WorkingStatus_e : int8_t
    {
        Error   = -1,
        Disable = 0,
        Normal  = 1,
        IsBusy  = 2,
        IsFull  = 3
    } WorkingStatus_e;

    /**
     * @brief 初始化结构体
     */
    typedef struct
    {
        // 设置区域大小，单位字
        struct ZoneSizeConfig_t
        {
            uint16_t notice_size = 0;
            uint16_t warning_size = 0;
            uint16_t error_size = 0;
            uint16_t fatal_size = 0;
            uint16_t def1_size = 0;
            uint16_t def2_size = 0;
            uint16_t def3_size = 0;
            uint16_t def4_size = 0;
        };

        bool             is_reset = false; // 是否重设
        ZoneSizeConfig_t zone_size;
    } Base_Config_t;

    class Base_c
    {
    public:
        virtual WorkingStatus_e Write(MessageType_e message_type, uint8_t *data, uint16_t len) = 0;
        virtual WorkingStatus_e Read(MessageType_e message_type, void *data, uint16_t start_index, uint16_t message_len) = 0;
        virtual WorkingStatus_e ResetConfig(Base_Config_t config) = 0;
        virtual WorkingStatus_e ClearAll() = 0;
        LogSaveType_e log_type() {return log_type_;}
        WorkingStatus_e working_statue() {return working_statue_;}
        uint16_t message_num(MessageType_e message_type) {return message_num_[(uint8_t)message_type];}

    protected:
        LogSaveType_e log_type_ = NoType;
        WorkingStatus_e working_statue_ = Disable;
        uint16_t message_num_[8] = {0,0,0,0,0,0,0,0};

    };
}

#endif