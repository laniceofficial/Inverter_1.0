/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_flash.cpp
 * @author  L_Zero
 * @version v26.1.1.0
 * @date    2025/12/6
 * @brief   最基础的flash操作封装
 *
 ********************************************************************************************
 * @attention
 *  flash的开始地址是0x08000000
 *  代码会从flash开始地址储存, 为避免操作到储存代码的flash, 我们一般只使用最后的扇区
 *  H732有4个扇区   4 * 128kb
 *  F405有12个扇区  4 * 16kb + 1 * 64kb + 7 * 128kb
 * 
 *  H7特性：每32位烧写时会生成1处ECC校验码并写入Flash无法修改，
 *          读取Flash时会计算ECC并且与Flash中储存的ECC比对，
 *          出现2处不同就进硬件中断；
 *          所以目前H7的Flash写入地址必须32位对齐并且每32位只能写一次(找不到其他解决办法)
 *  
 * @version
 *  v26.1.1.0 - 25/12/06
 *      取消H7上对写1字节、半字、字、2字函数的声明
 *  v26.1.1.0 - 25/12/08
 *      删除F4的写2字功能，恢复了H7写1字节、半字、1字的声明
 *
 ************************** Dongguan-University of Technology -ACE***************************/

#include "bsp_flash.hpp"
extern "C"
{
#include "string.h"
}

namespace BSP_n
{
    /**
     * @brief  读取flash数据
     * @param  addr     flash中被读取数据的起始地址
     * @param  buf      存储数据的起始地址
     * @param  len	    读取数据长度,以字节为单位
     */
    void Flash_ReadData(uint32_t addr, void *buf, uint16_t len)
    {
        memcpy(buf, (void *)addr, len);
    }

    /**
     * @brief  擦除扇区(将数据写入flash中原先已有数据的扇区前需要擦除扇区)
     * @param  SectorNum  需要擦除的目标扇区,取值范围 (注意不要占用已经被程序占用了的扇区)
     *                      H732为0~3   4 * 128kb
     *                      F405为0~11  4 * 16kb + 64kb + 7 * 128kb
     * @retval uint32_t	sectorError
     *					如果本次flash擦除产生了错误, 则发生擦除错误的页面号存储在SectorError中
     *
     * @attention 开始擦除到扇区擦除完成需要时间(100ms？没测过), 注意注意
     */
    uint32_t Flash_EraseSector(uint32_t SectorNum)
    {
        FLASH_EraseInitTypeDef FLASH_Erase;
        uint32_t sectorError = 0;

        HAL_FLASH_Unlock();
        FLASH_Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        FLASH_Erase.Banks = FLASH_BANK_1;                   // H732和F405都只有BANK1
        FLASH_Erase.Sector = SectorNum;                     // 扇区号
        FLASH_Erase.NbSectors = 1;                          // 这次操作擦除扇区数
        FLASH_Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 一次电平擦除量
        __disable_irq();
        HAL_FLASHEx_Erase(&FLASH_Erase, &sectorError);
        __enable_irq();
        HAL_FLASH_Lock();

        return sectorError;
    }

#if defined(STM32H7)
ALIGN_32BYTES(uint8_t flash_tx_buff[32]);     // 仅H7编程Flash需要的对齐8字节的uint8数组缓存区
#endif

    /**
     * @brief  将数据写入flash,写1字节
     * @param  addr  flash中写入数据的起始地址
     * @param  data  被写入数据的起始地址
     * @param  len   写入数据数量
     *
     * @retval HAL_StatusTypeDef 写入状态
     */
    HAL_StatusTypeDef Flash_WriteByte(uint32_t addr, uint8_t *data, uint16_t len)
    {
        HAL_StatusTypeDef status = HAL_ERROR;
#if defined(STM32H7)
        if((addr < FLASH_START_ADDR) || (((addr + len) + 31) & ~0x1FUL) > FLASH_END_ADDR + 1) // 字节对齐检测
        {
            return status;
        }
        memset(flash_tx_buff, 0xff, 32);
        if(memcmp((void *)addr, flash_tx_buff, 32) != 0)    // 重复写入检测
        {
            return status;
        }

        // 计算地址偏移与基地址
        uint8_t  offset_addr  = addr % 32;          // 偏移地址
        uint8_t  residue_addr = 32 - offset_addr;   // 剩余地址
        uint32_t base_addr    = addr - offset_addr; // 基地址

        // 处理起始非对齐部分
        if(len < residue_addr)
        {
            memcpy(flash_tx_buff + offset_addr, data, len); // 补充
            HAL_FLASH_Unlock();
            __disable_irq();
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, base_addr, (uint32_t)flash_tx_buff);    // 写
            __enable_irq();
            HAL_FLASH_Lock();
            return status;
        }
        else
        {
            memcpy(flash_tx_buff + offset_addr, data, residue_addr); // 补充
            HAL_FLASH_Unlock();
            __disable_irq();
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, base_addr, (uint32_t)flash_tx_buff);    // 写
            if(status != HAL_OK)
            {
                __enable_irq();
                HAL_FLASH_Lock();
                return status;
            }

            // 处理中间完整32字节块
            for(uint8_t i = 1; i * 32 + 32 < len + offset_addr; i++)
            {
                memcpy(flash_tx_buff, data + residue_addr + i * 32 - 32, 32);
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, base_addr + i * 32, (uint32_t)flash_tx_buff);
                if(status != HAL_OK)
                {
                    __enable_irq();
                    HAL_FLASH_Lock();
                    return status;
                }
            }

            // 处理末尾非完整块
            uint8_t residue_len = ((len + offset_addr) % 32); // 剩余未处理len
            memcpy(flash_tx_buff, data + len - residue_len, residue_len);
            memset(flash_tx_buff + residue_len, 0xff, 32 - residue_len);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, base_addr + (len / 32) * 32, (uint32_t)flash_tx_buff);
            __enable_irq();
            HAL_FLASH_Lock();
            return status;
        }

#elif defined(STM32F4)
        if(addr < FLASH_START_ADDR || addr + len > FLASH_END_ADDR + 1)
        {
            return status;
        }
        HAL_FLASH_Unlock();
        __disable_irq();
        for (uint16_t i = 0; i < len; i++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, data[i]);
        }
        __enable_irq();
        HAL_FLASH_Lock();
#endif
        return status;
    }

    /**
     * @brief  将数据写入flash,写半字
     * @param  addr  flash中写入数据的起始地址
     * @param  data  被写入数据的起始地址
     * @param  len   写入数据数量
     *
     * @retval HAL_StatusTypeDef 写入状态
     */
    HAL_StatusTypeDef Flash_WriteHalfWord(uint32_t addr, uint16_t *data, uint16_t len)
    {
        HAL_StatusTypeDef status = HAL_ERROR;
#if defined(STM32H7)
        status = Flash_WriteByte(addr, (uint8_t *)data, len * 2);
#elif defined(STM32F4)
        if(addr < FLASH_START_ADDR || addr + len * 2 > FLASH_END_ADDR + 1)
        {
            return status;
        }
        else if(addr % 2 != 0)
        {
            status = Flash_WriteByte(addr, (uint8_t *)data, len * 2);
            return status;
        }
        HAL_FLASH_Unlock();
        __disable_irq();
        for (uint16_t i = 0; i < len; i++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2 * i, data[i]);
            if(status != HAL_OK)
            {
                __enable_irq();
                HAL_FLASH_Lock();
                return status;
            }
        }
        __enable_irq();
        HAL_FLASH_Lock();
#endif
        return status;
    }

    /**
     * @brief  将数据写入flash,写1字
     * @param  addr  flash中写入数据的起始地址
     * @param  data  被写入数据的起始地址
     * @param  len   写入数据数量
     *
     * @retval HAL_StatusTypeDef 写入状态
     */
    HAL_StatusTypeDef Flash_WriteWord(uint32_t addr, uint32_t *data, uint16_t len)
    {
        HAL_StatusTypeDef status = HAL_ERROR;
#if defined(STM32H7)
        status = Flash_WriteByte(addr, (uint8_t *)data, len * 4);
#elif defined(STM32F4)
        if(addr < FLASH_START_ADDR || addr + len * WORD_2_BYTE > FLASH_END_ADDR + 1)
        {
            return status;
        }
        else if(addr % 4 != 0)
        {
            status = Flash_WriteByte(addr, (uint8_t *)data, len * WORD_2_BYTE);
            return status;
        }
        HAL_FLASH_Unlock();
        __disable_irq();
        for (uint16_t i = 0; i < len; i++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * WORD_2_BYTE, data[i]);
            if(status != HAL_OK)
            {
                __enable_irq();
                HAL_FLASH_Lock();
                return status;
            }
        }
        __enable_irq();
        HAL_FLASH_Lock();
#endif
        return status;
    }

#if defined(STM32H7)
    /**
     * @brief  将数据写入flash, H7一次写8字, 因为ECC原因一个地址只能写一次
     * @param  addr  flash中写入数据的起始地址, 因为ECC校验原因被迫32位对齐
     * @param  data  被写入数据的起始地址
     * @param  len   写入数据数量, 单位8字(32字节)
     *
     * @retval HAL_StatusTypeDef 写入状态
     */
    HAL_StatusTypeDef Flash_Write8Word(uint32_t addr, uint32_t *data, uint16_t len)
    {
        HAL_StatusTypeDef status = HAL_ERROR;
        if((addr % 32 != 0) || (addr < FLASH_START_ADDR) || (addr + len * 32 > FLASH_END_ADDR + 1)) // 字节对齐检测
        {
            return status;
        }
        memset(flash_tx_buff, 0xff, 32);
        if(memcmp((void *)addr, flash_tx_buff, 32) != 0)    // 重复写入检测
        {
            return status;
        }
        HAL_FLASH_Unlock();
        __disable_irq();
        for (uint16_t i = 0; i < len; i++)
        {
            memcpy(flash_tx_buff, (data + i * 32), 32);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + 32 * i, (uint32_t)flash_tx_buff);
            if(status != HAL_OK)
            {
                __enable_irq();
                HAL_FLASH_Lock();
                return status;
            }
        }
        __enable_irq();
        HAL_FLASH_Lock();
        return status;
    }
#endif

}