/**
 * (C)2019 Kevin Sangeelee.
 *
 * This file is licenced under the GNU GPL Version 2 or later.
 *     See https://www.gnu.org/licenses/licenses.html
 *
 * Use entirely at your own risk, or not at all.
 */

#include <stdio.h>
#include <tgmath.h>
#include "gd32vf103.h"
#include "systick.h"
#include "lcd.h"

#define BMP085_ADDRESS7     (0x77 << 1)

uint8_t eeprom[22] = { 0, 0, };
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

float pressure_hpa;

/**
 * Configure peripherals GPIOB and I2C0, which uses PB6 & PB7 as
 * open-drain outputs.
 */
void i2c_config(void)
{
    /* enable I2C0 and GPIOB clocks */
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_GPIOB);


    /* GPIO Pins 6 & 7, Alternate Function, Open Drain Outputs
       PB6 is I2C0_SCL   
       PB7 is I2C0_SDA
    */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    // Start with a reset of controller I2C0
    i2c_deinit(I2C0);

    // Now configure I2C0

    /* I2C clock configure */
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);

    /* enable I2C0 */
    i2c_enable(I2C0);
}

/*
 * Perform an I2C Start, and write the register address we want (e.g. the BMP085
 * Control Register is 0xF4)
 */

void bmp085_request_register(unsigned char addr)
{
    i2c_start_on_bus(I2C0);
    // SBSEND in in I2C_STAT0, clear when read.
    while( ! i2c_flag_get(I2C0, I2C_FLAG_SBSEND) );
    i2c_master_addressing(I2C0, BMP085_ADDRESS7, I2C_TRANSMITTER);

    while( ! i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) );
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

    // Ready to send, according to GD32V datasheet, p358
    i2c_data_transmit(I2C0, addr);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
}

/* 
 * Simply writes the given byte to the BMP085. If we requested the control
 * register previously, then this will be the value that we write to the
 * register.
 */
void bmp085_write_register(unsigned char value)
{
    i2c_data_transmit(I2C0, value);
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
}

/*
 * Reads 'count' bytes into the given buffer by issuing an I2C start condition
 * and then reading from the slave repeatedly until the required number of
 * bytes have been recieved.
 */
void bmp085_restart_read(unsigned char *buf, int count)
{
    i2c_start_on_bus(I2C0);
    while( ! i2c_flag_get(I2C0, I2C_FLAG_SBSEND) );
    i2c_master_addressing(I2C0, BMP085_ADDRESS7, I2C_RECEIVER);
    while( ! i2c_flag_get(I2C0, I2C_FLAG_ADDSEND) );
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

    i2c_ack_config(I2C0, I2C_ACK_ENABLE);

    int idx = 0;
    while(idx < count) {
        
        // Wait for Buffer Not Empty
        while(!i2c_flag_get(I2C0, I2C_FLAG_RBNE));

        // Take the byte from the buffer
        buf[idx++] = i2c_data_receive(I2C0);

        // Next byte is last? Then don't ACK.
        if(idx == count - 1) {
            i2c_ack_config(I2C0, I2C_ACK_DISABLE);
        }
    }
}

/*
 * Issue a stop condition on the bus, and wait for the controller to confirm.
 */
void i2c_stop()
{
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2C0) & I2C_CTL0_STOP);
}

/*
 * Reads the raw temperature and pressure values from the BMP085, and
 * calculates their compensated values from the calibration data that gets read
 * from eeprom.
 */
int read_bmp085(float altitude) {

    uint8_t pbuf[32]; // For sprintf.

    // Read eeprom data if the array is empty
    // I2C Device Address 0x77 (hardwired into the chip, 0xEE & 0xEF)
    if((unsigned short)*eeprom == 0) {
    
        // Device 0x77, register 0xaa, read into buf, 22 bytes
        bmp085_request_register(0xaa);
        bmp085_restart_read(eeprom, 22);
        i2c_stop();

        ac1 = (short)eeprom[0] << 8 | eeprom[1];
        ac2 = (short)eeprom[2] << 8 | eeprom[3];
        ac3 = (short)eeprom[4] << 8 | eeprom[5];
        ac4 = (unsigned short)eeprom[6] << 8 | eeprom[7];
        ac5 = (unsigned short)eeprom[8] << 8 | eeprom[9];
        ac6 = (unsigned short)eeprom[10] << 8 | eeprom[11];
        b1 = (short)eeprom[12] << 8 | eeprom[13];
        b2 = (short)eeprom[14] << 8 | eeprom[15];
        mb = (short)eeprom[16] << 8 | eeprom[17];
        mc = (short)eeprom[18] << 8 | eeprom[19];
        md = (short)eeprom[20] << 8 | eeprom[21];
    }
    
    
    uint8_t ut_buf[2];
    uint8_t up_buf[3];
        
    uint8_t cmd_ut = 0x2e;
    uint8_t cmd_up[] = {0x34, 0x74, 0xb4, 0xf4};
    int oss_delay[] = {4, 7, 13, 25};
    int oss = 1; // This is an index into the above array
    
    /*
     * Get Uncompensated Temperature from BMP085
     */
    bmp085_request_register(0xf4);
    bmp085_write_register(cmd_ut);
    i2c_stop();
    delay_1ms(5);
    bmp085_request_register(0xf6);
    bmp085_restart_read(ut_buf, 2);
    i2c_stop();
    
    long ut = (long)ut_buf[0] << 8 | ut_buf[1]; 
    
    // Temperature compensation algorithm (derived from datasheet)
    long x1 = ((ut - ac6) * ac5) >> 15;
    long x2 = (mc * (1 << 11)) / (x1 + md);
    long b5 = x1 + x2;
    long t = (b5 + 8)  >> 4;

    sprintf((char *)pbuf, "Tem: %d.%dC", (int)t/10, (int)t % 10);
    LCD_ShowString8(0, 6 * 8, pbuf, YELLOW);
    
    float p0 = 0;
    
    /*
     * Get Uncompensated Pressure from BMP085, based on the OverSampling Setting
     * of (0, 1, 2, or 3). This determines accuracy, conversion delay, and power consumption.
     */
    bmp085_request_register(0xf4);
    bmp085_write_register(cmd_up[oss]);
    i2c_stop();
    delay_1ms(oss_delay[oss]); // just wait the maximum possible time for conversion
    bmp085_request_register(0xf6);
    bmp085_restart_read(up_buf, 3);
    i2c_stop();

    long up = (((long)up_buf[0] << 16) | ((long)up_buf[1] << 8) | up_buf[2]) >> (8 - oss);
        
    // Pressure compensation algorithm (derived from datasheet)
    long b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    long x3 = x1 + x2;
    long b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    unsigned long b4 = ac4 * (unsigned long)(x3 + 32768) >>15;
    unsigned long b7 = ((unsigned long)up - b3) * (50000 >> oss);
    long p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + (x1 + x2 + 3791) / 16;
        
    p0 = (float)p / powf(1.0f - (altitude / 44330), 5.255f);
        
    sprintf((char *)pbuf, "PrU: %d.%02d hPa", (int)(p/100), (int)(p % 100));
    LCD_ShowString8(0, 7 * 8, pbuf, YELLOW);

    pressure_hpa = p0 / 100;
    sprintf((char *)pbuf, "Pr: %d hPa\n", (int)pressure_hpa);
    LCD_ShowString8(0, 8 * 8, pbuf, YELLOW);
    
    return 0;
}

