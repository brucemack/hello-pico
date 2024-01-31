/*
Hello World PI PICO

# Make sure that the PICO_SDK_PATH is set properly

cd /home/bruce/pico/hello-pico
# You need main.c and CMakeLists.txt
cp ../pico-sdk/external/pico_sdk_import.cmake .
mkdir build
cd build
cmake ..
make
# Connect the PI PICO to USB while pressing the BOOTSEL button.
# And then:
cp test.uf2 /media/bruce/RPI-RP2

# Make sure the SWD is connected properly:
# GPIO24 (Pin 18) to SWDIO
# GPIO25 (Pin 22) to SWCLK
# GND (Pin 20) to SWGND

# Use this command to flash:
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program test.elf verify reset exit"

# Looking at the serial port:
minicom -b 115200 -o -D /dev/ttyACM0
*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define I2C0_SDA 4 // Pin 6: I2C channel 0 - data
#define I2C0_SCL 5 // Pin 7: I2C channel 0 - clock

const uint LED_PIN = 25;

bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
 
    bi_decl(bi_program_description("This is a test binary."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

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
    
    while (1) {
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
        gpio_put(LED_PIN, 1);
        //puts("Hello World\n");
        sleep_ms(1000);
    }
}

