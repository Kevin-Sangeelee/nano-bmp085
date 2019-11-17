/*!
    \file  main.c
    \brief master receiver
    
    \version 2019-6-5, V1.0.0, firmware for GD32VF103

*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include <stdio.h>
#include "lcd.h"
//#include "gd32vf103v_eval.h"

#define I2C_10BIT_ADDRESS      0

#define I2C0_OWN_ADDRESS7      0x72
#define I2C0_SLAVE_ADDRESS7    (0x77 << 1)

uint8_t i2c_receiver[16];

void rcu_config(void);
void i2c_config(void);
void bmp085_request_register(unsigned char);
void bmp085_restart_read(unsigned char *, int);
void i2c_stop(void);
int read_bmp085(float altitude);


char hexval(unsigned char val)
{
    if(val < 10)
        return '0' + val;
    return 'A' + (val - 10);
}

/*
 * Print the 32-bit hex value to LCD at the specified line number.
 * The value is shown in big-endian form.
 */
void printHex(unsigned char line, unsigned long val) {

    char buf[10];

    for(int i=0; i < 4; i++) {
        buf[i]   = hexval((val >> (28 - (i << 2))) & 0xf);
        buf[i+5] = hexval((val >> (12 - (i << 2))) & 0xf);
    }
    buf[4] = ':';
    buf[9] = 0;

    LCD_ShowString8(0, (line << 3), (u8 *)(buf), YELLOW);
}

/*****************************
    main function
*/
int main(void)
{
    /* Configure peripheral clocks, and the GPIO & I2C peripherals */
    rcu_config();
    gpio_init(GPIOA, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
    i2c_config();

    Lcd_Init(); // Init SPI0 and the attached LCD
    LCD_Clear(BLACK);
    LCD_ShowString(0, 0, (unsigned char *)"I2C_Test", YELLOW);

    unsigned char eeprom[22];

    i2c_config();

    while(1) {

        bmp085_request_register(0xaa);
        bmp085_restart_read(eeprom, 22);
        i2c_stop();

        printHex(0, eeprom[ 0] << 24 | eeprom[ 1] << 16 | eeprom[ 2] << 8 | eeprom[ 3]);
        printHex(1, eeprom[ 4] << 24 | eeprom[ 5] << 16 | eeprom[ 6] << 8 | eeprom[ 7]);
        printHex(2, eeprom[ 8] << 24 | eeprom[ 9] << 16 | eeprom[10] << 8 | eeprom[11]);
        printHex(3, eeprom[12] << 24 | eeprom[13] << 16 | eeprom[14] << 8 | eeprom[15]);
        printHex(4, eeprom[16] << 24 | eeprom[17] << 16 | eeprom[18] << 8 | eeprom[19]);
        printHex(5, eeprom[20] << 24 | eeprom[21] << 16);

        read_bmp085(213);

        //while(gpio_input_bit_get(GPIOA, 8));

        delay_1ms(5000);
    }

    while(1){
    }
}

/*!
    Enable the peripheral clock
*/
void rcu_config(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
}

void i2c_test(void)
{
    unsigned char eeprom[22];

    i2c_config();
    bmp085_request_register(0xaa);

    bmp085_restart_read(eeprom, 22);

    i2c_stop();
}
