/*
 * sd.h
 *
 *  Created on: 28 èþë. 2021 ã.
 *      Author: mango
 */

/***************
 * SPI1
 ***************/

#ifndef INC_SD_H_
#define INC_SD_H_

#define CS_SD_GPIO_PORT GPIOA
#define CS_SD_PIN GPIO_PIN_8
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET)
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET)

/*
 * LED
 */
//#define LD_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
//#define LD_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

//--------------------------------------------------

/* Card type flags (CardType) */
#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1 | CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */
//--------------------------------------------------


#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


typedef struct sd_info
{
	volatile uint8_t type_SD;
}	sd_info_ptr;

uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);

uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);

uint8_t sd_ini(void);

void SD_PowerOn(void);

void SPI_Release(void);

uint8_t SPI_wait_ready(void);

uint8_t SPI_ReceiveByte(void);

static uint8_t SD_cmd (uint8_t cmd, uint32_t arg);

#endif /* INC_SD_H_ */
