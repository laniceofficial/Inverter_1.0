#ifndef __LOCAL_FLASH_LOG_HPP
#define __LOCAL_FLASH_LOG_HPP

#include "log_base.hpp"
#include "bsp_flash.hpp"

#define LOCAL_FLASH_CONFIG_USE_SIZE 9  // 配置需要的大小（单位 word）
#define LOCAL_FLASH_MESSAGE_ADD     2   // 消息附加大小 （单位 word）
#if defined(STM32H7) // H723VET 512K
#define LOCAL_FLASH_START_ADDR 0x08060000
#define LOCAL_FLASH_END_ADDR   0x0807FFFF
#define LOCAL_FLASH_USE_SECTOR 3
#elif defined(STM32F4) // H405RGT 1024K
#define LOCAL_FLASH_START_ADDR 0x080E0000
#define LOCAL_FLASH_END_ADDR   0x080FFFFF
#define LOCAL_FLASH_USE_SECTOR 11
#endif

namespace Log_n
{
    class LocalFlash_c : public Base_c
    {
    public:
        struct ZoneAddr_t
        {
            uint32_t notice;
            uint32_t warning;
            uint32_t error;
            uint32_t fatal;
            uint32_t def1;
            uint32_t def2;
            uint32_t def3;
            uint32_t def4;
            uint32_t end = LOCAL_FLASH_END_ADDR;

            ZoneAddr_t operator-(const ZoneAddr_t& instance) const;
            ZoneAddr_t operator++ () const;
            uint32_t operator[] (MessageType_e message_type);
        };

        static LocalFlash_c* Get_InstancePtr();
        static LocalFlash_c* Get_InstancePtr(Base_Config_t config);
        static void Init(Base_Config_t config);

        WorkingStatus_e Write(MessageType_e message_type, uint8_t *data, uint16_t len) override;
        WorkingStatus_e Read(MessageType_e message_type, void *data, uint16_t start_index, uint16_t message_num) override;
        WorkingStatus_e ResetConfig(Base_Config_t config) override;
        WorkingStatus_e ClearAll() override;

        const ZoneAddr_t& zone_start_addr() { return zone_start_addr_;}
        const ZoneAddr_t& now_addr() { return now_addr_;}

    private:
        ZoneAddr_t zone_start_addr_ = {};
        ZoneAddr_t now_addr_ = {};
        uint32_t now_message_addr_ = 0;
        uint32_t next_message_addr_ = 0;
        uint8_t local_flash_buff_[32];

        LocalFlash_c(Base_Config_t config);
    };
}

#endif //! __LOCAL_FLASH_LOG_HPP
