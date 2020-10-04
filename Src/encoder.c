/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "gpio.h"
#include "uart_ext.h"

static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);


// returns true for even number of ones (no parity error according to AS5047 datasheet
uint8_t spi_check_parity(uint16_t x) {
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (~x) & 1;
}

/**
 * Timer interrupt
 */
void encoder_read(void) {
	uint16_t pos;

		spi_begin();
		spi_transfer(&pos, 0, 1);
		spi_end();
    //pos &= 0x3FFF;
    uprintf("%d\r\n",pos);
    
}



// Software SPI
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

            HAL_GPIO_WritePin(ENCODER_CLK_GPIO_Port,ENCODER_CLK_Pin,GPIO_PIN_SET);

			spi_delay();
            
			int samples = 0;
			samples += HAL_GPIO_ReadPin(ENCODER_MISO_GPIO_Port,ENCODER_MISO_Pin);
			__NOP();
			samples += HAL_GPIO_ReadPin(ENCODER_MISO_GPIO_Port,ENCODER_MISO_Pin);
			__NOP();
			samples += HAL_GPIO_ReadPin(ENCODER_MISO_GPIO_Port,ENCODER_MISO_Pin);
			__NOP();
			samples += HAL_GPIO_ReadPin(ENCODER_MISO_GPIO_Port,ENCODER_MISO_Pin);
			__NOP();
			samples += HAL_GPIO_ReadPin(ENCODER_MISO_GPIO_Port,ENCODER_MISO_Pin);

			recieve <<= 1;
			if (samples > 2) {
				recieve |= 1;
			}

			HAL_GPIO_WritePin(ENCODER_CLK_GPIO_Port,ENCODER_CLK_Pin,GPIO_PIN_RESET);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
    HAL_GPIO_WritePin(ENCODER_CSN_GPIO_Port,ENCODER_CSN_Pin,GPIO_PIN_RESET);
}

static void spi_end(void) {
	HAL_GPIO_WritePin(ENCODER_CSN_GPIO_Port,ENCODER_CSN_Pin,GPIO_PIN_SET);;
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
