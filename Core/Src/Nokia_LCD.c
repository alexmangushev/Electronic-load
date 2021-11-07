/*
 * Nokia_LCD.c
 *
 *  Created on: Jul 16, 2021
 *      Author: mango
 */

#include "main.h"
#include "NOKIA_LCD.h"

/*#define RST 3
#define CLK 2
#define DIN 1
#define DC 0*/
#define LCD_X 84
#define LCD_Y 48

const uint8_t ASCII[][5] =
{
 {0x00, 0x00, 0x00, 0x00, 0x00} // 20
,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ?
,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e <
,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f >

,{0xFF, 0x81, 0x83, 0x86, 0xFC} // 80 sd detected
,{0x00, 0x00, 0x00, 0x00, 0x00} // 81
,{0x00, 0x00, 0x00, 0x00, 0x00} // 82
,{0x00, 0x00, 0x00, 0x00, 0x00} // 83
,{0x00, 0x00, 0x00, 0x00, 0x00} // 84
,{0x00, 0x00, 0x00, 0x00, 0x00} // 85
,{0x00, 0x00, 0x00, 0x00, 0x00} // 86
,{0x00, 0x00, 0x00, 0x00, 0x00} // 87
,{0x00, 0x00, 0x00, 0x00, 0x00} // 88
,{0x00, 0x00, 0x00, 0x00, 0x00} // 89
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8a
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8b
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8c
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8d
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8e
,{0x00, 0x00, 0x00, 0x00, 0x00} // 8f

,{0x00, 0x00, 0x00, 0x00, 0x00} // 90
,{0x00, 0x00, 0x00, 0x00, 0x00} // 91
,{0x00, 0x00, 0x00, 0x00, 0x00} // 92
,{0x00, 0x00, 0x00, 0x00, 0x00} // 93
,{0x00, 0x00, 0x00, 0x00, 0x00} // 94
,{0x00, 0x00, 0x00, 0x00, 0x00} // 95
,{0x00, 0x00, 0x00, 0x00, 0x00} // 96
,{0x00, 0x00, 0x00, 0x00, 0x00} // 97
,{0x00, 0x00, 0x00, 0x00, 0x00} // 98
,{0x00, 0x00, 0x00, 0x00, 0x00} // 99
,{0x00, 0x00, 0x00, 0x00, 0x00} // 9a
,{0x00, 0x00, 0x00, 0x00, 0x00} // 9b
,{0x00, 0x00, 0x00, 0x00, 0x00} // 9c
,{0x00, 0x00, 0x00, 0x00, 0x00} // 9d
,{0x00, 0x00, 0x00, 0x00, 0x00} // 9e
,{0x00, 0x00, 0x00, 0x00, 0x00}// 9f

,{0x00, 0x00, 0x00, 0x00, 0x00} // a0
,{0x00, 0x00, 0x00, 0x00, 0x00} // a1
,{0x00, 0x00, 0x00, 0x00, 0x00} // a2
,{0x00, 0x00, 0x00, 0x00, 0x00} // a3
,{0x00, 0x00, 0x00, 0x00, 0x00} // a4
,{0x00, 0x00, 0x00, 0x00, 0x00} // a5
,{0x00, 0x00, 0x00, 0x00, 0x00} // a6
,{0x00, 0x00, 0x00, 0x00, 0x00} // a7
,{0x00, 0x00, 0x00, 0x00, 0x00} // a8
,{0x00, 0x00, 0x00, 0x00, 0x00} // a9
,{0x00, 0x00, 0x00, 0x00, 0x00} // aa
,{0x00, 0x00, 0x00, 0x00, 0x00} // ab
,{0x00, 0x00, 0x00, 0x00, 0x00} // ac
,{0x00, 0x00, 0x00, 0x00, 0x00} // ad
,{0x00, 0x00, 0x00, 0x00, 0x00} // ae
,{0x00, 0x00, 0x00, 0x00, 0x00}// af

,{0x00, 0x00, 0x00, 0x00, 0x00} // b0
,{0x00, 0x00, 0x00, 0x00, 0x00} // b1
,{0x00, 0x00, 0x00, 0x00, 0x00} // b2
,{0x00, 0x00, 0x00, 0x00, 0x00} // b3
,{0x00, 0x00, 0x00, 0x00, 0x00} // b4
,{0x00, 0x00, 0x00, 0x00, 0x00} // b5
,{0x00, 0x00, 0x00, 0x00, 0x00} // b6
,{0x00, 0x00, 0x00, 0x00, 0x00} // b7
,{0x00, 0x00, 0x00, 0x00, 0x00} // b8
,{0x00, 0x00, 0x00, 0x00, 0x00} // b9
,{0x00, 0x00, 0x00, 0x00, 0x00} // ba
,{0x00, 0x00, 0x00, 0x00, 0x00} // bb
,{0x00, 0x00, 0x00, 0x00, 0x00} // bc
,{0x00, 0x00, 0x00, 0x00, 0x00} // bd
,{0x00, 0x00, 0x00, 0x00, 0x00} // be
,{0x00, 0x00, 0x00, 0x00, 0x00},// bf

{ 0x7C, 0x12, 0x11, 0x12, 0x7C },  // c0 �
{ 0x7F, 0x49, 0x49, 0x49, 0x31 },  // c1 �
{ 0x7F, 0x49, 0x49, 0x49, 0x36 },  // c2 �
{ 0x7F, 0x01, 0x01, 0x01, 0x01 },  // c3 �
{ 0x60, 0x3F, 0x21, 0x3F, 0x60 },  // c4 �
{ 0x7F, 0x49, 0x49, 0x49, 0x41 },  // c5 �
{ 0x77, 0x08, 0x7F, 0x08, 0x77 },  // c6 �
{ 0x22, 0x41, 0x49, 0x49, 0x36 },  // c7 �
{ 0x7F, 0x10, 0x08, 0x04, 0x7F },  // c8 �
{ 0x7E, 0x10, 0x09, 0x04, 0x7E },  // c9 �
{ 0x7F, 0x08, 0x14, 0x22, 0x41 },  // ca �
{ 0x40, 0x3E, 0x01, 0x01, 0x7F },  // cb �
{ 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // cc �
{ 0x7F, 0x08, 0x08, 0x08, 0x7F },  // cd �
{ 0x3E, 0x41, 0x41, 0x41, 0x3E },  // ce �
{ 0x7F, 0x01, 0x01, 0x01, 0x7F },  // cf �
{ 0x7F, 0x09, 0x09, 0x09, 0x06 },  // d0 �
{ 0x3E, 0x41, 0x41, 0x41, 0x22 },  // d1 �
{ 0x01, 0x01, 0x7F, 0x01, 0x01 },  // d2 �
{ 0x07, 0x48, 0x48, 0x48, 0x3F },  // d3 �
{ 0x0E, 0x11, 0x7F, 0x11, 0x0E },  // d4 �
{ 0x63, 0x14, 0x08, 0x14, 0x63 },  // d5 �
{ 0x3F, 0x20, 0x20, 0x3F, 0x60 },  // d6 �
{ 0x07, 0x08, 0x08, 0x08, 0x7F },  // d7 �
{ 0x7F, 0x40, 0x7E, 0x40, 0x7F },  // d8 �
{ 0x3F, 0x20, 0x3F, 0x20, 0x7F },  // d9 �
{ 0x01, 0x7F, 0x48, 0x48, 0x30 },  // da �
{ 0x7F, 0x48, 0x30, 0x00, 0x7F },  // db �
{ 0x00, 0x7F, 0x48, 0x48, 0x30 },  // dc �
{ 0x22, 0x41, 0x49, 0x49, 0x3E },  // dd �
{ 0x7F, 0x08, 0x3E, 0x41, 0x3E },  // de �
{ 0x46, 0x29, 0x19, 0x09, 0x7F },  // df �
{ 0x20, 0x54, 0x54, 0x54, 0x78 },  // e0 �
{ 0x3C, 0x4A, 0x4A, 0x4A, 0x31 },  // e1 �
{ 0x7C, 0x54, 0x54, 0x28, 0x00 },  // e2 �
{ 0x7C, 0x04, 0x04, 0x0C, 0x00 },  // e3 �
{ 0x60, 0x3C, 0x24, 0x3C, 0x60 },  // e4 �
{ 0x38, 0x54, 0x54, 0x54, 0x18 },  // e5 �
{ 0x6C, 0x10, 0x7C, 0x10, 0x6C },  // e6 �
{ 0x00, 0x44, 0x54, 0x54, 0x28 },  // e7 �
{ 0x7C, 0x20, 0x10, 0x08, 0x7C },  // e8 �
{ 0x7C, 0x21, 0x12, 0x09, 0x7C },  // e9 �
{ 0x7C, 0x10, 0x28, 0x44, 0x00 },  // ea �
{ 0x40, 0x38, 0x04, 0x04, 0x7C },  // eb �
{ 0x7C, 0x08, 0x10, 0x08, 0x7C },  // ec �
{ 0x7C, 0x10, 0x10, 0x10, 0x7C },  // ed �
{ 0x38, 0x44, 0x44, 0x44, 0x38 },  // ee �
{ 0x7C, 0x04, 0x04, 0x04, 0x7C },  // ef �
{ 0x7C, 0x14, 0x14, 0x14, 0x08 },  // f0 �
{ 0x38, 0x44, 0x44, 0x44, 0x00 },  // f1 �
{ 0x04, 0x04, 0x7C, 0x04, 0x04 },  // f2 �
{ 0x0C, 0x50, 0x50, 0x50, 0x3C },  // f3 �
{ 0x08, 0x14, 0x7C, 0x14, 0x08 },  // f4 �
{ 0x44, 0x28, 0x10, 0x28, 0x44 },  // f5 �
{ 0x3C, 0x20, 0x20, 0x3C, 0x60 },  // f6 �
{ 0x0C, 0x10, 0x10, 0x10, 0x7C },  // f7 �
{ 0x7C, 0x40, 0x7C, 0x40, 0x7C },  // f8 �
{ 0x3C, 0x20, 0x3C, 0x20, 0x7C },  // f9 �
{ 0x04, 0x7C, 0x50, 0x50, 0x20 },  // fa �
{ 0x7C, 0x50, 0x20, 0x00, 0x7C },  // fb �
{ 0x00, 0x7C, 0x50, 0x50, 0x20 },  // fc �
{ 0x28, 0x44, 0x54, 0x54, 0x38 },  // fd �
{ 0x7C, 0x10, 0x38, 0x44, 0x38 },  // fe �
{ 0x48, 0x54, 0x34, 0x14, 0x7C }   // ff �
};

void send_byte(uint8_t data)  //send byte to display
{
    for (uint8_t i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, 0);  //CLK 0

        if (data & (1<<7) )
        	HAL_GPIO_WritePin(DIN_GPIO_Port, DIN_Pin, 1);
        else
        	HAL_GPIO_WritePin(DIN_GPIO_Port, DIN_Pin, 0);

        data <<= 1;

        HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, 1);  //CLK 1
    }
}

void display_cmd(uint8_t data)  //send command to display
{
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 0); // DC 0
    send_byte(data);
}

void display_data(uint8_t data) //send data to display
{
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1);  // DC 1
    send_byte(data);
}

void display_init(void)  //initialization of display
{
	 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);  //RST 0
	 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);  //RST 1
     display_cmd(0x21);	// extended command set
     display_cmd(0xB1);	// offset voltage
     display_cmd(0x04);	// Temperature correction mode 0
     display_cmd(0x14);	// offset scheme 1:48
     //display_cmd(0xB7);
     display_cmd(0xB8);
     //display_cmd(0xB6);//0xB6 - offset voltage
     display_cmd(0x20);
     display_cmd(0x0c); // Normal show(1 - black pixel, 0 - none)
}

void display_clear(void) //clear display
{
    for (uint32_t i = 0; i < LCD_X*LCD_Y/8; i++)
	{
		display_data(0x00);
	}
}

void display_string(uint8_t* ch) //send string to display
{
    while (*ch)
    {
        display_data(0x00);

        for (uint8_t i = 0; i < 5; i++)
        {
            display_data(ASCII[*ch-0x20][i]);
        }

        display_data(0x00);
        ch++;
    }
}

void display_char(uint8_t ch)  //send char to display from ASCII table
{
	display_data(0x00);

	for (uint8_t i = 0; i < 5; i++)
	{
		display_data(ASCII[ch-0x20][i]);
	}

	display_data(0x00);
}

/*
 * 	0-y-11
 * 	|
 * 	x
 * 	|
 * 	5
 */
void display_set_cursor(uint8_t x, uint8_t y) //set cursor on coordinates
{
    display_cmd(0x40+y);
    display_cmd(0x80+7*x);
}

void display_char_underline(uint8_t ch)  //send underline char
{
	display_data(0x00);

	for (uint8_t i = 0; i < 5; i++)
	{
		display_data(ASCII[ch-0x20][i] | 0x80);
	}

	display_data(0x00);
}
