/*
 * Nokia_LCD.h
 *
 *  Created on: Jul 16, 2021
 *      Author: mango
 */

#include "main.h"

#ifndef INC_NOKIA_LCD_H_
#define INC_NOKIA_LCD_H_

void send_byte(uint8_t data);  //отправить байт в дисплей

void display_cmd(uint8_t data);  //отправить команду в дисплей

void display_data(uint8_t data); //отправить данные в дисплей

// main functions for using
void display_init(void);  //инициализация дисплея

void display_clear(void); //очистка дисплея

void display_string(uint8_t* ch); //отправить строку в дисплей

void display_char(uint8_t ch);  //отправить символ в дисплей

void display_char_underline(uint8_t ch);  //отправить подчеркнутый символ в дисплей

void display_set_cursor(uint8_t x, uint8_t y); //установить курсор на координаты

#endif /* INC_NOKIA_LCD_H_ */
