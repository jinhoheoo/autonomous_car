/*
 * liquidcrystal_i2c.h
 *
 *  Created on: Feb 5, 2022
 *      Author: Nicky Kim
 *  Copyright (c) 2022 KiMSON All rights reserved.
 *
 * STM32 HAL library for LCD display based on 16x2 CLCD with PCF8574
 * Currently, the "stm32f4xx_hal.h" library is used.
 * You must include and use the appropriate library for your board.
 *
 */

#ifndef INC_LIQUIDCRYSTAL_I2C_H_
#define INC_LIQUIDCRYSTAL_I2C_H_

#include "stm32f4xx_hal.h"
//#include "stm32l0xx_hal.h"
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit

#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);

#endif /* INC_LIQUIDCRYSTAL_I2C_H_ */
