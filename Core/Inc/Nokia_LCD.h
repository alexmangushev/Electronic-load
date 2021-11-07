/*
 * Nokia_LCD.h
 *
 *  Created on: Jul 16, 2021
 *      Author: mango
 */

#include "main.h"

#ifndef INC_NOKIA_LCD_H_
#define INC_NOKIA_LCD_H_

void send_byte(uint8_t data);  //��������� ���� � �������

void display_cmd(uint8_t data);  //��������� ������� � �������

void display_data(uint8_t data); //��������� ������ � �������

// main functions for using
void display_init(void);  //������������� �������

void display_clear(void); //������� �������

void display_string(uint8_t* ch); //��������� ������ � �������

void display_char(uint8_t ch);  //��������� ������ � �������

void display_char_underline(uint8_t ch);  //��������� ������������ ������ � �������

void display_set_cursor(uint8_t x, uint8_t y); //���������� ������ �� ����������

#endif /* INC_NOKIA_LCD_H_ */
