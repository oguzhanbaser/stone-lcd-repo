/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

UnbufferedSerial serial_port(PA_2, PA_3);
AnalogIn pot1(PA_0), pot2(PA_1);
DigitalOut mLED(PC_13);

uint8_t lcd_buffer[255];
uint16_t adcVal1[32], adcVal2[32];
int readCnt = 20, cnt = 0;

Timer t1;

void clear_buffer()
{
    for(int i = 0; i < 255; i++)
    {
        lcd_buffer[i] = 0;
    }
}

// add point to one chart
void add_point(uint8_t pCh, uint8_t pLen, const uint16_t *p1Value){
    uint8_t mCnt = 0;   

    lcd_buffer[mCnt++] = 0xA5;              // buffer header 1
    lcd_buffer[mCnt++] = 0x5A;              // buffer header 2
    lcd_buffer[mCnt++] = (pLen * 2) + 2;    // data length
    lcd_buffer[mCnt++] = 0x84;              // chart data header
    lcd_buffer[mCnt++] = pCh;               // chart channel number

    // chart value
    for (int i = 0; i < pLen; i++) {
        lcd_buffer[mCnt++] = (p1Value[i] & 0xFF00) >> 8;    // 16 bit data MSB
        lcd_buffer[mCnt++] = p1Value[i] & 0x00FF;           // 16 bit data LSB
    }

    serial_port.write(lcd_buffer, mCnt);    // send data buffer to LCD
    clear_buffer();                         // clear data buffer
}

// add point to two charts same time
void add_point_multi(uint8_t pLen, const uint16_t *p1Value, const uint16_t *p2Value) {
    uint8_t mCnt = 0;

    lcd_buffer[mCnt++] = 0xA5;               // buffer header 1
    lcd_buffer[mCnt++] = 0x5A;               // buffer header 2
    lcd_buffer[mCnt++] = (pLen * 2 * 2) + 2; // data length
    lcd_buffer[mCnt++] = 0x84;               // chart data header
    lcd_buffer[mCnt++] = 0x03;               // chart channel number

    for (int i = 0; i < pLen; i++) {
        lcd_buffer[mCnt++] = (p1Value[i] & 0xFF00) >> 8;        // 16 bit first data MSB
        lcd_buffer[mCnt++] = p1Value[i] & 0x00FF;               // 16 bit first data LSB

        lcd_buffer[mCnt++] = (p2Value[i] & 0xFF00) >> 8;        // 16 bit second data MSB
        lcd_buffer[mCnt++] = p2Value[i] & 0x00FF;               // 16 bit second data LSB
        
    }

    serial_port.write(lcd_buffer, mCnt);        // send data buffer to LCD
    clear_buffer();                             // clear data buffer
}

// main() runs in its own thread in the OS
int main() {
  serial_port.baud(115200);
  mLED = 1;
  t1.start();

  while (true) {

    // if elapset time bigger than 10ms save adc values to arrays
    if(t1.elapsed_time() > 10ms)
    {
        //read adc values
        float nAdcVal1 = pot1.read();       
        float nAdcVal2 = pot2.read();       

        // multiply adc value to scale data 
        adcVal1[cnt] = nAdcVal1 * 500;
        adcVal2[cnt] = nAdcVal2 * 500;
        cnt++;

        // reset timer
        t1.reset();
    }

    // if adc read count equals to buffer limit draw chart
    if(cnt == readCnt)
    {
        // add_point(1, readCnt, adcVal1);              // uncomment if you want to draw one chart
        add_point_multi(readCnt, adcVal1, adcVal2);     // uncomment if you want to draw two charts same time
        mLED = !mLED;                                   // toggle status LED
        cnt = 0;                                        // reset counter
    }

    thread_sleep_for(1);
  }
}
