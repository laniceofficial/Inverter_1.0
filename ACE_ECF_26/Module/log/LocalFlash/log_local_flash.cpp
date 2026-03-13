/************************** Dongguan-University of Technology -ACE**************************
 * @file  log_local_flash.hpp
 * @brief 本地flash做日志
 * @author L_Zero
 * @version v26.1.1.0
 * @note
 *  *  因为本地flash就一个所以搞单例
 *  *  本地flash-log遵循以下协议
 *  *  *  Flash被分为9个区域(config, Notice, Warning, Error, Fatal, UserDef1~4), 各区域flash空间相互连接
 *  *  *  起始9字用来储存log配置(8组区域对应起始地址与最终结束地址), 存储在config区
 *  *  *  其他区域分别可以存储信息(message), 每条信息第一个uint32_t记录下一条信息首地址, 信息的最后一个uint32_t记录此信息的首地址
 ****************************************************************************************************************
 *
 * @version v26.1.1.0 -- 25/12/09
 *  *  添加读取成员变量zone_start_addr_，now_addr_的函数
 *  *  重载ZoneAddr_t结构体的"-" "++"运算符，现在可以使用特定方法对结构体做运算获得想要的实例
 *  *  修改初始化逻辑，现在配置空间超出上限不再初始化失败，而是自动缩小空间
 *
 *
************************** Dongguan-University of Technology -ACE***************************/

#include "log_local_flash.hpp"

extern "C"
{
#include "string.h"
}

namespace Log_n
{
    /* region ================================ 工具 ================================ */
#define fCHECK_NEED_NORMAL if(working_statue_ != WorkingStatus_e::Normal){return working_statue_;}
#define fCHECK_NOT_BUSY if(working_statue_ == WorkingStatus_e::IsBusy){return WorkingStatus_e::IsBusy;}
#define fALIGN_8WORD(x) (((x) + 31) & ~0x1FUL) // 向上对齐8字
#define fMIN(x, y) (x < y ? x : y)

    // endregion



    /* region ================================ 单例构造与初始化 ================================ */
    LocalFlash_c* local_flash_instance = nullptr;            // 唯一实例的指针
    uint32_t flash_config_buff[LOCAL_FLASH_CONFIG_USE_SIZE]; // 写入flash/从flash读取 的配置

    /**
     * @brief 获取单例指针, 在未实例化时会返回nullptr
     *
     */
    LocalFlash_c* LocalFlash_c::Get_InstancePtr()
    {
        return local_flash_instance;
    }

    /**
     * @brief 获取单例指针, 在未实例化时会实例化并返回
     * @param config 记录初始化参数
     */
    LocalFlash_c* LocalFlash_c::Get_InstancePtr(Base_Config_t config)
    {
        if(local_flash_instance == nullptr)
        {
            local_flash_instance = new LocalFlash_c(config);
        }
        return local_flash_instance;
    }

    /**
     * @brief 初始化函数, 调用就生单例
     * @param config 记录初始化参数
     */
    void LocalFlash_c::Init(Base_Config_t config)
    {
        if(local_flash_instance == nullptr)
        {
            local_flash_instance = new LocalFlash_c(config);
        }
    }

    /**
     * @brief 私有构造, 为了生单例
     * @param config 记录初始化参数
     * @attention config.is_reset 如果设为false将会忽视config中的其他配置, 使用flash中下载的配置
     */
    LocalFlash_c::LocalFlash_c(Base_Config_t config)
    {
        log_type_ = LocalFlash;
        memset(message_num_, 0, 32);
        if(config.is_reset) // 重烧配置
        {
            // 处理新配置
#if defined(STM32F4)
            zone_start_addr_.notice  = LOCAL_FLASH_START_ADDR + LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE;
            zone_start_addr_.warning = fMAX(zone_start_addr_.notice + config.zone_size.notice_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.error = fMAX(zone_start_addr_.warning + config.zone_size.warning_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.fatal = fMAX(zone_start_addr_.error + config.zone_size.error_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def1 = fMAX(zone_start_addr_.fatal + config.zone_size.fatal_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def2 = fMAX(zone_start_addr_.def1 + config.zone_size.def1_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def3 = fMAX(zone_start_addr_.def2 + config.zone_size.def2_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def4 = fMAX(zone_start_addr_.def3 + config.zone_size.def3_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.end = fMAX(zone_start_addr_.def4 + config.zone_size.def4_size * WORD_2_BYTE, LOCAL_FLASH_END_ADDR + 1);
#elif defined(STM32H7)
            zone_start_addr_.notice = fMIN(fALIGN_8WORD(LOCAL_FLASH_START_ADDR + LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.warning = fMIN(fALIGN_8WORD(zone_start_addr_.notice + config.zone_size.notice_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.error = fMIN(fALIGN_8WORD(zone_start_addr_.warning + config.zone_size.warning_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.fatal = fMIN(fALIGN_8WORD(zone_start_addr_.error + config.zone_size.error_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def1 = fMIN(fALIGN_8WORD(zone_start_addr_.fatal + config.zone_size.fatal_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def2 = fMIN(fALIGN_8WORD(zone_start_addr_.def1 + config.zone_size.def1_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def3 = fMIN(fALIGN_8WORD(zone_start_addr_.def2 + config.zone_size.def2_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.def4 = fMIN(fALIGN_8WORD(zone_start_addr_.def3 + config.zone_size.def3_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
            zone_start_addr_.end = fMIN(fALIGN_8WORD(zone_start_addr_.def4 + config.zone_size.def4_size * WORD_2_BYTE), LOCAL_FLASH_END_ADDR + 1);
#endif
            // 擦除旧配置
            BSP_n::Flash_EraseSector(LOCAL_FLASH_USE_SECTOR);

            // 烧写新配置
            memcpy(flash_config_buff, &zone_start_addr_, LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE);
            BSP_n::Flash_WriteWord(LOCAL_FLASH_START_ADDR, flash_config_buff, LOCAL_FLASH_CONFIG_USE_SIZE);

            // 获取新配置
            BSP_n::Flash_ReadData(LOCAL_FLASH_START_ADDR, &now_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE);

            // 对比检查
            if(memcmp(&now_addr_, &zone_start_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE) != 0)
            {
                working_statue_ = WorkingStatus_e::Error;
                return;
            }
        }
        else // 使用flash里的配置
        {
            // 获取配置
            BSP_n::Flash_ReadData(LOCAL_FLASH_START_ADDR, &zone_start_addr_, LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE);
            memcpy(flash_config_buff, &zone_start_addr_, LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE);

            // 检索当前写到的地址值
            for(uint8_t i = 0; i < LOCAL_FLASH_CONFIG_USE_SIZE - 1; i++)
            {
                if(flash_config_buff[i] < LOCAL_FLASH_START_ADDR || flash_config_buff[i] > LOCAL_FLASH_END_ADDR)
                {
                    working_statue_ = WorkingStatus_e::IsFull; // 地址越界
                    return;
                }
                while (*((uint32_t *)flash_config_buff[i]) != 0xFFFFFFFF) // 未擦干净也会Error
                {
                    if(flash_config_buff[i] > flash_config_buff[i+1])
                    {
                        working_statue_ = WorkingStatus_e::IsFull; // 地址越界
                        return;
                    }
                    flash_config_buff[i] = *((uint32_t *)flash_config_buff[i]);
                    message_num_[i]++;
                    if(flash_config_buff[i] < LOCAL_FLASH_START_ADDR || flash_config_buff[i] > LOCAL_FLASH_END_ADDR)
                    {
                        working_statue_ = WorkingStatus_e::IsFull; // 地址越界
                        return;
                    }
                }
            }
            memcpy(&now_addr_, flash_config_buff, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE);
        }
        working_statue_ = WorkingStatus_e::Normal;
    }
    // endregion



    /* region ================================ 继承函数重写 ================================ */
    /**
     * @brief 写入信息
     * @param message_type 信息类型
     * @param data 存储数据指针
     * @param len 数据长度(单位字节)
     * @return WorkingStatus_e 写入状态
     */
    WorkingStatus_e LocalFlash_c::Write(MessageType_e message_type, uint8_t *data, uint16_t len)
    {
        fCHECK_NEED_NORMAL;
#if defined(STM32F4)
        // 获取下次信息首地址与本信息首地址
        now_message_addr_ = ((uint32_t *)&now_addr_)[(uint8_t)message_type];
        next_message_addr_ = now_message_addr_ + len + LOCAL_FLASH_MESSAGE_ADD * WORD_2_BYTE;

        // 检查越界
        if(next_message_addr_ > ((uint32_t *)&zone_start_addr_)[(uint8_t)message_type + 1])
        {
            return WorkingStatus_e::IsFull;
        }

        // 开写开写
        working_statue_ = WorkingStatus_e::IsBusy;
        BSP_n::Flash_WriteWord(now_message_addr_, &next_message_addr_, 1);  // 首地址指向内存存下次信息首地址
        BSP_n::Flash_WriteByte(now_message_addr_ + WORD_2_BYTE, data, len); // 之后存信息
        BSP_n::Flash_WriteWord(next_message_addr_ - WORD_2_BYTE, &now_message_addr_, 1); // 信息尾地址指向内存存本信息首地址
        message_num_[(uint8_t)message_type]++; // 更新储存信息数
        ((uint32_t *)&now_addr_)[(uint8_t)message_type] = next_message_addr_; // 更新当前写信息地址
        working_statue_ = WorkingStatus_e::Normal;
#elif defined(STM32H7) // 因为H7的ECC纠错机制导致每32字节只能写一次，读写方法与F4不同
        // 获取下次信息首地址与本信息首地址
        now_message_addr_ = ((uint32_t *)&now_addr_)[(uint8_t)message_type];
        next_message_addr_ = fALIGN_8WORD(now_message_addr_ + len + LOCAL_FLASH_MESSAGE_ADD * WORD_2_BYTE);

        // 检查越界
        if(next_message_addr_ > ((uint32_t *)&zone_start_addr_)[(uint8_t)message_type + 1])
        {
            return WorkingStatus_e::IsFull;
        }

        // 开写开写
        memset(local_flash_buff_, 0xff, 28);
        working_statue_ = WorkingStatus_e::IsBusy;
        if(len + LOCAL_FLASH_MESSAGE_ADD * WORD_2_BYTE < 32)
        {
            // 填充数据包
            memcpy(local_flash_buff_, &next_message_addr_, WORD_2_BYTE);
            memcpy(local_flash_buff_ + WORD_2_BYTE, data, len);
            memcpy(local_flash_buff_ + 28, &now_message_addr_, WORD_2_BYTE);
            BSP_n::Flash_Write8Word(now_message_addr_, (uint32_t*)local_flash_buff_, 1);
        }
        else
        {
            memcpy(local_flash_buff_, &next_message_addr_, WORD_2_BYTE);
            if(len > 28)
            {
                // 填充首数据并写入
                memcpy(local_flash_buff_ + WORD_2_BYTE, data, 28);
                BSP_n::Flash_Write8Word(now_message_addr_, (uint32_t*)local_flash_buff_, 1);

                // 写入中段32位对齐的数据(如果有)
                for(uint16_t i = 1; i * 32 <= len - 28; i++)
                {
                    BSP_n::Flash_Write8Word(now_message_addr_ + i * 32, (uint32_t *)(data + i * 32 - WORD_2_BYTE), 1);
                }

                // 填充尾数据并写入
                memset(local_flash_buff_, 0xff, 28);
                uint8_t tail_len = (len - 28) / 32;
                memcpy(local_flash_buff_, data + len - tail_len, tail_len);
                if(tail_len + 4 < 32) // 尾数据无需分包
                {
                    memcpy(local_flash_buff_ + 28, &now_message_addr_, 4);
                    BSP_n::Flash_Write8Word(next_message_addr_ - 32, (uint32_t*)local_flash_buff_, 1);
                }
                else // 尾数据需要分包
                {
                    BSP_n::Flash_Write8Word(next_message_addr_ - 64, (uint32_t*)local_flash_buff_, 1);
                    memset(local_flash_buff_, 0xff, 28);
                    memcpy(local_flash_buff_ + 28, &now_message_addr_, 4);
                    BSP_n::Flash_Write8Word(next_message_addr_ - 32, (uint32_t*)local_flash_buff_, 1);
                }
            }
            else
            {
                // 填充数据包
                memcpy(local_flash_buff_ + WORD_2_BYTE, data, len);
                BSP_n::Flash_Write8Word(now_message_addr_, (uint32_t*)local_flash_buff_, 1);
            }
            working_statue_ = WorkingStatus_e::Normal;
        }
        message_num_[(uint8_t)message_type]++; // 更新储存信息数
        ((uint32_t *)&now_addr_)[(uint8_t)message_type] = next_message_addr_; // 更新当前写信息地址
        working_statue_ = WorkingStatus_e::Normal;
#endif
        return working_statue_;
    }

    /**
     * @brief 读信息
     * @param message_type 信息类型
     * @param data 接收信息数组指针
     * @param start_index 从第几条信息开始读(默认第一条信息的索引是0)
     * @param message_num 读几条信息
     * @return WorkingStatus_e 读取状态
     *
     * @attention 不会检查data大小, 警惕data数组越界
     */
    WorkingStatus_e LocalFlash_c::Read(MessageType_e message_type, void *data, uint16_t start_index, uint16_t message_num)
    {
        fCHECK_NEED_NORMAL;
        if(start_index + message_num > message_num_[(uint8_t)message_type])
        {
            return WorkingStatus_e::IsFull; // 超出可阅读量
        }

        working_statue_ = WorkingStatus_e::IsBusy;
        // 找到start_index对应信息首地址
        if(start_index / 2 < message_num_[(uint8_t)message_type] - start_index) // 从前往后找
        {
            now_message_addr_ = ((uint32_t *)&zone_start_addr_)[(uint8_t)message_type];
            for(uint16_t i = 0; i < start_index; i++)
            {
                now_message_addr_ = *(uint32_t *)now_message_addr_;
            }
        }
        else // 从后往前找
        {
            now_message_addr_ = ((uint32_t *)&now_addr_)[(uint8_t)message_type] - WORD_2_BYTE;
            for(uint16_t i = 0; i < message_num_[(uint8_t)message_type] - start_index; i++)
            {
                now_message_addr_ = *(uint32_t *)(now_message_addr_ - WORD_2_BYTE);
            }
        }

        // 开读开读
        uint8_t* now_read = (uint8_t*)data;
        for(uint16_t i = 0; i < message_num; i++)
        {
            next_message_addr_ = *(uint32_t *)now_message_addr_;
            BSP_n::Flash_ReadData(now_message_addr_ + WORD_2_BYTE, now_read, next_message_addr_ - now_message_addr_ - LOCAL_FLASH_MESSAGE_ADD * WORD_2_BYTE);
            now_read += (next_message_addr_ - now_message_addr_ - LOCAL_FLASH_MESSAGE_ADD * WORD_2_BYTE);
#if defined(STM32H7) // 对于H7，目前采用这种办法去除数据尾带有多余数据
            while(*(--now_read) == 0xFFu){} // 问题1：如果数据末尾带0xFF也会被一起删掉
            now_read++;                     // 问题2：数据尾的0xFF无法擦除
#endif
            now_message_addr_ = next_message_addr_;
        }
        working_statue_ = WorkingStatus_e::Normal;

        return working_statue_;
    }

    /**
     * @brief 重新配置flash, 此操作会清空log
     * @param config 重设的配置
     * @return WorkingStatus_e 重设情况
     */
    WorkingStatus_e LocalFlash_c::ResetConfig(Log_n::Base_Config_t config)
    {
        fCHECK_NOT_BUSY;

        // 处理新配置
#if defined(STM32F4)
        zone_start_addr_.notice  = LOCAL_FLASH_START_ADDR + LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE;
        zone_start_addr_.warning = zone_start_addr_.notice + config.zone_size.notice_size * WORD_2_BYTE;
        zone_start_addr_.error   = zone_start_addr_.warning + config.zone_size.warning_size * WORD_2_BYTE;
        zone_start_addr_.fatal   = zone_start_addr_.error + config.zone_size.error_size * WORD_2_BYTE;
        zone_start_addr_.def1    = zone_start_addr_.fatal + config.zone_size.fatal_size * WORD_2_BYTE;
        zone_start_addr_.def2    = zone_start_addr_.def1 + config.zone_size.def1_size * WORD_2_BYTE;
        zone_start_addr_.def3    = zone_start_addr_.def2 + config.zone_size.def2_size * WORD_2_BYTE;
        zone_start_addr_.def4    = zone_start_addr_.def3 + config.zone_size.def3_size * WORD_2_BYTE;
        zone_start_addr_.end     = zone_start_addr_.def4 + config.zone_size.def4_size * WORD_2_BYTE;
#elif defined(STM32H7)
        zone_start_addr_.notice  = fALIGN_8WORD(LOCAL_FLASH_START_ADDR + LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE);
        zone_start_addr_.warning = fALIGN_8WORD(zone_start_addr_.notice + config.zone_size.notice_size * WORD_2_BYTE);
        zone_start_addr_.error   = fALIGN_8WORD(zone_start_addr_.warning + config.zone_size.warning_size * WORD_2_BYTE);
        zone_start_addr_.fatal   = fALIGN_8WORD(zone_start_addr_.error + config.zone_size.error_size * WORD_2_BYTE);
        zone_start_addr_.def1    = fALIGN_8WORD(zone_start_addr_.fatal + config.zone_size.fatal_size * WORD_2_BYTE);
        zone_start_addr_.def2    = fALIGN_8WORD(zone_start_addr_.def1 + config.zone_size.def1_size * WORD_2_BYTE);
        zone_start_addr_.def3    = fALIGN_8WORD(zone_start_addr_.def2 + config.zone_size.def2_size * WORD_2_BYTE);
        zone_start_addr_.def4    = fALIGN_8WORD(zone_start_addr_.def3 + config.zone_size.def3_size * WORD_2_BYTE);
        zone_start_addr_.end     = fALIGN_8WORD(zone_start_addr_.def4 + config.zone_size.def4_size * WORD_2_BYTE);
#endif
        // 检查是否越界
        if(zone_start_addr_.end > LOCAL_FLASH_END_ADDR + 1)
        {
            working_statue_ = Error;
            return WorkingStatus_e::Error;
        }

        // 擦除旧配置
        BSP_n::Flash_EraseSector(LOCAL_FLASH_USE_SECTOR);
        memset(message_num_, 0, 32);

        // 烧写新配置
        memcpy(flash_config_buff, &zone_start_addr_, LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE);
        BSP_n::Flash_WriteWord(LOCAL_FLASH_START_ADDR, flash_config_buff, LOCAL_FLASH_CONFIG_USE_SIZE);

        // 获取新配置
        BSP_n::Flash_ReadData(LOCAL_FLASH_START_ADDR, &now_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE);

        // 对比检查
        if(memcmp(&now_addr_, &zone_start_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE) != 0)
        {
            working_statue_ = WorkingStatus_e::Error;
        }
        else
        {
            working_statue_ = WorkingStatus_e::Normal;
        }
        return working_statue_;
    }

    /**
     * @brief 清除所有log, 不会清除配置
     * @return WorkingStatus_e 清除状态
     */
    WorkingStatus_e LocalFlash_c::ClearAll()
    {
        fCHECK_NEED_NORMAL;
        BSP_n::Flash_EraseSector(LOCAL_FLASH_USE_SECTOR); // 擦全部
        memset(message_num_, 0, 32);
        memcpy(flash_config_buff, &zone_start_addr_, LOCAL_FLASH_CONFIG_USE_SIZE * WORD_2_BYTE); // 配置导入缓存
        BSP_n::Flash_WriteWord(LOCAL_FLASH_START_ADDR, flash_config_buff, LOCAL_FLASH_CONFIG_USE_SIZE); // 重写配置
        BSP_n::Flash_ReadData(LOCAL_FLASH_START_ADDR, &now_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE); // 载入新配置
        if(memcmp(&now_addr_, &zone_start_addr_, (LOCAL_FLASH_CONFIG_USE_SIZE - 1) * WORD_2_BYTE) != 0) // 对比检查
        {
            working_statue_ = WorkingStatus_e::Error;
        }
        return working_statue_;
    }
    // endregion



    /* region ================================ 功能函数 ================================ */

    // 重载 "实例 - 实例" 定义为实例的成员相减
    LocalFlash_c::ZoneAddr_t LocalFlash_c::ZoneAddr_t::operator- (const LocalFlash_c::ZoneAddr_t& instance) const
    {
        LocalFlash_c::ZoneAddr_t used_space;
        used_space.notice = this->notice - instance.notice;
        used_space.warning = this->warning - instance.warning;
        used_space.error = this->error - instance.error;
        used_space.fatal = this->fatal - instance.fatal;
        used_space.def1 = this->def1 - instance.def1;
        used_space.def2 = this->def2 - instance.def2;
        used_space.def3 = this->def3 - instance.def3;
        used_space.def4 = this->def4 - instance.def4;
        used_space.end = 0;
        return used_space;
    }

    // 重载 "++实例" 定义为成员内容前移
    LocalFlash_c::ZoneAddr_t LocalFlash_c::ZoneAddr_t::operator++ () const
    {
        LocalFlash_c::ZoneAddr_t zone;
        zone.notice = this->warning;
        zone.warning = this->error;
        zone.error = this->fatal;
        zone.fatal = this->def1;
        zone.def1 = this->def2;
        zone.def2 = this->def3;
        zone.def3 = this->def4;
        zone.def4 = this->end;
        zone.end = this->end;
        return zone;
    }
    // endregion
}
