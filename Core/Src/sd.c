/*
 * sd.c
 *
 *  Created on: 28 июл. 2021 г.
 *      Author: mango
 */
#include "sd.h"

/***************
 * SPI1
 ***************/

extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef huart2;

//--------------------------------------------------

// Definitions for MMC/SDC command

#define CMD0 (0x40+0) // GO_IDLE_STATE
#define CMD1 (0x40+1) // SEND_OP_COND (MMC)
#define ACMD41 (0xC0+41) // SEND_OP_COND (SDC)
#define CMD8 (0x40+8) // SEND_IF_COND
#define CMD9 (0x40+9) // SEND_CSD
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR

sd_info_ptr sdinfo;
//char str1[60] = {};

//-----------------------------------------------
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba)
{
	uint8_t result;
	uint16_t cnt;

	result = SD_cmd (CMD17, lba); //CMD17 даташит стр 50 и 96
	if (result != 0x00)
		return 5; //Выйти, если результат не 0x00

	SPI_Release();

	cnt = 0;

	do //Ждем начала блока
	{
		result = SPI_ReceiveByte();
		cnt++;
	} while ( (result!=0xFE) && (cnt<0xFFFF) );

	if (cnt >= 0xFFFF) return 5;

	for (cnt = 0; cnt < 512; cnt++)
		buff[cnt] = SPI_ReceiveByte(); //получаем байты блока из шины в буфер

	SPI_Release(); //Пропускаем контрольную сумму

	SPI_Release();
	return 0;
}
//-----------------------------------------------

//-----------------------------------------------

uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba)
{
	uint8_t result;
	uint16_t cnt;

	result = SD_cmd(CMD24, lba); //CMD24 даташит стр 51 и 97-98

	if (result != 0x00) //Выйти, если результат не 0x00
		return 6;

	SPI_Release();

	SPI_SendByte (0xFE); //Начало буфера

	for (cnt=0; cnt < 512; cnt++)
		SPI_SendByte(buff[cnt]); //Данные

	SPI_Release(); //Пропустим котрольную сумму

	SPI_Release();

	result = SPI_ReceiveByte();

	if ((result&0x05) != 0x05)
		return 6; //Выйти, если результат не 0x05 (Даташит стр 111)

	cnt = 0;

	do //Ждем окончания состояния BUSY
	{
		result = SPI_ReceiveByte();
		cnt++;
	} while ( (result!=0xFF)&&(cnt<0xFFFF) );

	if (cnt>=0xFFFF)
		return 6;

	return 0;
}

//-----------------------------------------------

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;

	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		//Error();
	}
	return receivedbyte;
}

void SPI_SendByte(uint8_t byte)
{
	SPIx_WriteRead(byte);
}
//-----------------------------------------------

uint8_t SPI_ReceiveByte(void)
{
	uint8_t bt = SPIx_WriteRead(0xFF);
	return bt;
}

//-----------------------------------------------
static uint8_t SD_cmd (uint8_t cmd, uint32_t arg)
{
	uint8_t n, res;

	// ACMD<n> is the command sequense of CMD55-CMD<n>
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		res = SD_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	// Select the card
	SS_SD_DESELECT();
	SPI_ReceiveByte();
	SS_SD_SELECT();
	SPI_ReceiveByte();
	// Send a command packet

	SPI_SendByte(cmd); // Start + Command index
	SPI_SendByte((uint8_t)(arg >> 24)); // Argument[31..24]
	SPI_SendByte((uint8_t)(arg >> 16)); // Argument[23..16]
	SPI_SendByte((uint8_t)(arg >> 8)); // Argument[15..8]
	SPI_SendByte((uint8_t)arg); // Argument[7..0]

	n = 0x01; // Dummy CRC + Stop
	if (cmd == CMD0) {n = 0x95;} // Valid CRC for CMD0(0)
	if (cmd == CMD8) {n = 0x87;} // Valid CRC for CMD8(0x1AA)
	SPI_SendByte(n);

	// Receive a command response
	n = 10; // Wait for a valid response in timeout of 10 attempts
	do
	{
	  res = SPI_ReceiveByte();
	} while ((res & 0x80) && --n);

	return res;
}
//-----------------------------------------------


void SD_PowerOn(void)
{
	HAL_Delay(20); //ждём 20 милисекунд, для того, чтобы напряжение стабилизировалось
	return;
}

uint8_t sd_ini(void)
{
	uint8_t i, cmd;
	uint8_t ocr[4];
	int16_t tmr;
	uint32_t temp;

	sdinfo.type_SD = 0;

	temp = hspi1.Init.BaudRatePrescaler;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //156.25 kbbs
	HAL_SPI_Init(&hspi1);

	SS_SD_DESELECT();

	for(i = 0; i < 10; i++) //80 импульсов (не менее 74) Даташит стр 91
		SPI_Release();

	hspi1.Init.BaudRatePrescaler = temp;
	HAL_SPI_Init(&hspi1);

	SS_SD_SELECT();

	if (SD_cmd(CMD0, 0) == 1) // Enter Idle state
	{
		SPI_Release();

		if (SD_cmd(CMD8, 0x1AA) == 1) // SDv2
		{
			for (i = 0; i < 4; i++)
				ocr[i] = SPI_ReceiveByte();
			//sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\n",ocr[0],ocr[1],ocr[2],ocr[3]);
			//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);

			// Get trailing return value of R7 resp
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) // The card can work at vdd range of 2.7-3.6V
			{
				for (tmr = 12000; tmr && SD_cmd(ACMD41, 1UL << 30); tmr--); // Wait for leaving idle state (ACMD41 with HCS bit)

				if (tmr && SD_cmd(CMD58, 0) == 0) // Check CCS bit in the OCR
				{
					for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
					//sprintf(str1,"OCR: 0x%02X 0x%02X 0x%02X 0x%02X\n",ocr[0],ocr[1],ocr[2],ocr[3]);
					//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
					sdinfo.type_SD = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)

				}
			}
		}
		else //SDv1 or MMCv3
		{
			if (SD_cmd(ACMD41, 0) <= 1)
			{
				sdinfo.type_SD = CT_SD1; cmd = ACMD41; // SDv1
			}
			else
			{
				sdinfo.type_SD = CT_MMC; cmd = CMD1; // MMCv3
			}

			for (tmr = 25000; tmr && SD_cmd(cmd, 0); tmr--) ; // Wait for leaving idle state
			if (!tmr || SD_cmd(CMD16, 512) != 0) // Set R/W block length to 512

			sdinfo.type_SD = 0;
		}
	}
	else
	{
	    return 1;
	}

    //sprintf(str1,"Type SD: 0x%02X\n",sdinfo.type_SD);
	//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);

	return 0;
}

//-----------------------------------------------

void SPI_Release(void)
{
	SPIx_WriteRead(0xFF);
}

uint8_t SPI_wait_ready(void)
{
  uint8_t res;
  uint16_t cnt;
  cnt=0;

  do { //Ждем окончания состояния BUSY
    res = SPI_ReceiveByte();
    cnt++;
  } while ( (res!=0xFF)&&(cnt<0xFFFF) );

  if (cnt>=0xFFFF) return 1;

  return res;

}
