#ifndef __BSP_FLASH_HPP
#define __BSP_FLASH_HPP

#include "main.h"

#define WORD_2_BYTE 4
#define FLASH_START_ADDR 0x08000000UL
#if defined(STM32F4)
#define FLASH_END_ADDR 0x080FFFFFUL
#elif defined(STM32H7)
#define FLASH_END_ADDR 0x0807FFFFUL
#endif

namespace BSP_n
{
    void Flash_ReadData(uint32_t addr, void *buf, uint16_t len);
    uint32_t Flash_EraseSector(uint32_t SectorNum);
    HAL_StatusTypeDef Flash_WriteByte(uint32_t addr, uint8_t *data, uint16_t len);
    HAL_StatusTypeDef Flash_WriteHalfWord(uint32_t addr, uint16_t *data, uint16_t len);
    HAL_StatusTypeDef Flash_WriteWord(uint32_t addr, uint32_t *data, uint16_t len);
#if defined(STM32H7)
    HAL_StatusTypeDef Flash_Write8Word(uint32_t addr, uint32_t *data, uint16_t len);
#endif
}

#endif //! __BSP_FLASH_HPP