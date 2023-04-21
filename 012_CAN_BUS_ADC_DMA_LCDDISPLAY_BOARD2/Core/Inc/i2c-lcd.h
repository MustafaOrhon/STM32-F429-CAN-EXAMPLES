/*
 * i2c-lcd.h
 *
 *  Created on: May 22, 2022
 *      Author: Mustafa
 */
#include "stm32f4xx_hal.h"
#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_
void lcd_init (void);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_send_string (char *str);
void lcd_clear(void);


#endif /* INC_I2C_LCD_H_ */
