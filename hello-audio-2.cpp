/*
Load command:
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program hello-audio-2.elf verify reset exit"

Minicom (for console, not the UART being tested):
minicom -b 115200 -o -D /dev/ttyACM0
*/
#include <cstdint>
#include <iostream>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
 
#define LED_PIN (25)
#define I2C0_SDA (4) // Phy Pin 6: I2C channel 0 - data
#define I2C0_SCL (5) // Phy Pin 7: I2C channel 0 - clock

using namespace std;

static const float PI = 3.1415926;

static const uint32_t pcmDataLen = 512;
static uint16_t pcmData[pcmDataLen];
static uint32_t outPtr = 0;

bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void makeFrame0(uint8_t* buf, uint16_t data) {
    buf[0] = 0x40;
    buf[1] = (data >> 4);
    buf[2] = (uint8_t)(data << 4);
}

void makeFrame1(uint8_t* buf, uint16_t data) {
    buf[0] = 0x0f & (data >> 8);
    buf[1] = data & 0xff;
}

int main() {
 
    stdio_init_all();

    // Stock LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Setup I2C
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    cout << "Hello, Audio Test 2" << endl;

    uint32_t p = 0;
    float phi = 0.0;
    float omega = 2.0 * PI / (float)pcmDataLen;
    for (uint32_t i = 0; i < pcmDataLen; i++) {  
        float a = (std::cos(phi) + 1.0) / 2.0;
        pcmData[p] = 4000.0 * a;
        cout << i << " " << a << " " << pcmData[p] << endl;
        phi += omega;
        p++;
    }
    /*
    // I2C SCAN
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    for (int addr = 0; addr < (1 << 7); ++addr) {
      if (addr % 16 == 0) {
        printf("%02x ", addr);
      }
      int ret;
      uint8_t rxdata;
      if (reserved_addr(addr))
        ret = PICO_ERROR_GENERIC;
      else
        ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);

      printf(ret < 0 ? "." : "@");
      printf(addr % 16 == 15 ? "\n" : "  ");
    }
    */

    // WORKS:
    i2c_set_baudrate(i2c_default, 400000 * 3);

    uint8_t dac_addr = 0x60;
    uint8_t buf[3];

    uint16_t i = 0;

    while (1) {
        //makeFrame0(buf, pcmData[i]);
        //i2c_write_blocking(i2c_default, dac_addr, buf, 3, true);
        makeFrame1(buf, pcmData[i]);
        i2c_write_blocking(i2c_default, dac_addr, buf, 2, true);
        i += 8;
        if (i == 512) {
             i = 0;
        }
    }
}
