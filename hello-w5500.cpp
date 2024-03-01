#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define LED_PIN (25)

// W5500 Module MISO
#define SPI0_RX_PIN (4)
// W5500 Module MISO
#define SPI0_TX_PIN (3)
#define SPI0_SCK_PIN (2)
#define SPI0_CSN_PIN (5)
#define W5500_INT_PIN (6)
#define W5500_RST_PIN (7)

using namespace std;

/*
Build:
    cd build
    cmake ..
    make hello-w5500

Load:
    openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program hello-w5500.elf verify reset exit"

Minicom (for console, not the UART being tested):
minicom -b 115200 -o -D /dev/ttyACM0
*/
int main() {
 
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
       
    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    cout << "Hello, W5500 test" << endl;

    spi_init(spi0, 500 * 1000);
    gpio_set_function(SPI0_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI0_TX_PIN, GPIO_FUNC_SPI);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(SPI0_CSN_PIN);
    gpio_set_dir(SPI0_CSN_PIN, GPIO_OUT);
    gpio_put(SPI0_CSN_PIN, 1);
    // Reset is active low
    gpio_init(W5500_RST_PIN);
    gpio_set_dir(W5500_RST_PIN, GPIO_OUT);
    gpio_put(W5500_RST_PIN, 1);
    // Interrupt input
    gpio_init(W5500_INT_PIN);
    gpio_set_dir(W5500_INT_PIN, GPIO_IN);

    // Do the formal reset
    gpio_put(W5500_RST_PIN, 0);
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_RST_PIN, 1);

    sleep_ms(10);

    // Ask for chip version
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");

    uint8_t buf[8];
    // Offset
    buf[0] = 0;
    buf[1] = 0x39;
    // Control
    buf[2] = 0;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 1);

    gpio_put(SPI0_CSN_PIN, 1);

    cout << "Version " << (int)buf[0] << endl;

    while (1) {
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
}

