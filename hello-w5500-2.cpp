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
    make hello-w5500-2

Load:
    openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program hello-w5500-2.elf verify reset exit"

Minicom (for console, not the UART being tested):
minicom -b 115200 -o -D /dev/ttyACM0
*/

void prettyHexDump(const uint8_t* data, uint32_t len, std::ostream& out,
    bool color);

static void set_Sn_reg8(spi_inst_t* spi, uint8_t socket, uint16_t offset, uint8_t v) {
    uint8_t buf[4];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 1) << 3) | 0b1'00;
    // Data 
    buf[3] = v;
    spi_write_blocking(spi0, buf, 4);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void set_Sn_reg16(spi_inst_t* spi, uint8_t socket, uint16_t offset, uint16_t v) {
    uint8_t buf[5];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 1) << 3) | 0b1'00;
    // Data 
    buf[3] = (v & 0xff00) >> 8;
    buf[4] = (v & 0x00ff);
    spi_write_blocking(spi0, buf, 5);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void set_Sn_MR(spi_inst_t* spi, uint8_t socket, uint8_t v) {
    set_Sn_reg8(spi, socket, 0x00, v);
}

static void set_Sn_CR(spi_inst_t* spi, uint8_t socket, uint8_t v) {
    set_Sn_reg8(spi, socket, 0x01, v);
}

static void set_Sn_IR(spi_inst_t* spi, uint8_t socket, uint8_t v) {
    set_Sn_reg8(spi, socket, 0x02, v);
}

static void set_Sn_RXBUF_SIZE(spi_inst_t* spi, uint8_t socket, uint8_t v) {
    set_Sn_reg8(spi, socket, 0x1e, v);
}

static void set_Sn_TXBUF_SIZE(spi_inst_t* spi, uint8_t socket, uint8_t v) {
    set_Sn_reg8(spi, socket, 0x1f, v);
}

static void set_Sn_RX_RD(spi_inst_t* spi, uint8_t socket, uint16_t v) {
    set_Sn_reg16(spi, socket, 0x28, v);
}

static uint8_t get_Sn_reg8(spi_inst_t* spi, uint8_t socket, uint16_t offset) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 1) << 3) | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 1);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    return buf[0];
}    

static uint16_t get_Sn_reg16(spi_inst_t* spi, uint8_t socket, uint16_t offset) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 1) << 3) | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 2);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    return (buf[0] << 8) | buf[1];
}    

static void get_Sn_RXBUF(spi_inst_t* spi, uint8_t socket, uint16_t offset, uint8_t* buf2, uint16_t len2) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 3) << 3) | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf2, len2);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}    

static uint8_t get_Sn_IR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg8(spi, socket, 0x02);
}    

static uint8_t get_Sn_SR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg8(spi, socket, 0x03);
}    

static uint16_t get_Sn_RX_RSR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x26);
}    

static uint16_t get_Sn_RX_RD(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x28);
}    

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

    sleep_ms(1);

    // Do the formal reset
    gpio_put(W5500_RST_PIN, 0);
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_RST_PIN, 1);

    sleep_ms(10);
    //uint8_t buf[8];

    // Adjust buffer size up to maximum
    set_Sn_RXBUF_SIZE(spi0, 0, 16);
    set_Sn_TXBUF_SIZE(spi0, 0, 16);
    // S0_MR Setup (MACRAW)
    set_Sn_MR(spi0, 0, 0b1001'0100);
    // S0_CR Setup (OPEN)
    set_Sn_CR(spi0, 0, 0x01);
    // Wait for status = OPEN
    while (true) {
        if (get_Sn_SR(spi0, 0) == 0x42) {
            break;
        }
        sleep_ms(50);
    }

    // Clear interrupts
    set_Sn_IR(spi0, 0, 0xff);

    const uint16_t blockSize = 16384;
    const uint16_t blockMask = blockSize - 1;
    int blocks = 32;

    // Receive loop
    while (1) {     
        uint8_t ir = get_Sn_IR(spi0, 0);
        if (ir & 0x04) {
            set_Sn_IR(spi0, 0, 0xff);
            // Look at data 
            uint16_t rx_rsr = get_Sn_RX_RSR(spi0, 0);
            uint16_t rx_rd = get_Sn_RX_RD(spi0, 0);            
            // Compute the reads, possibly in two parts if we are 
            // close to the end of the buffer
            uint16_t offset0 = rx_rd & blockMask;
            uint16_t len0 = std::min(rx_rsr, (uint16_t)(blockSize - offset0));
            uint16_t offset1 = 0;
            uint16_t len1 = rx_rsr - len0;

            uint8_t buf[blockSize];
            get_Sn_RXBUF(spi0, 0, offset0, buf, len0);
            if (len1 > 0) {
                get_Sn_RXBUF(spi0, 0, offset1, buf + len0, len1);
            }
            cout << "--------------------------------------------------" << endl;
            prettyHexDump(buf, len0 + len1, cout, true);

            // Note that the management of RX_RD is independent of the 
            // size of the RX buffer.  The addressing just pretends
            // everything is 64K.
            // This is expected to wrap at 0xffff!  
            rx_rd += rx_rsr;
            set_Sn_RX_RD(spi0, 0, rx_rd);
            // Tell the controller that we've done a receive
            set_Sn_CR(spi0, 0, 0x40);

            blocks--;
            if (blocks == 0) {
                break;
            }
        }

        sleep_ms(100);
    }

    while(true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
}

void prettyHexDump(const uint8_t* data, uint32_t len, std::ostream& out,
    bool color) {
    
    uint32_t lines = len / 16;
    if (len % 16 != 0) {
        lines++;
    }

    char buf[16];

    for (uint32_t line = 0; line < lines; line++) {

        // Position counter
        int l = snprintf(buf, 16, "%04X | ", (unsigned int)line * 16);
        if (l > 16) {
            panic("OVERFLOW");
        }
        out << buf;

        // Hex section
        for (uint16_t i = 0; i < 16; i++) {
            uint32_t k = line * 16 + i;
            if (k < len) {
                sprintf(buf, "%02x", (unsigned int)data[k]);
                out << buf << " ";
            } else {
                out << "   ";
            }
            if (i == 7) {
                out << " ";
            }
        }
        // Space between hex and ASCII section
        out << " ";

        if (color) {   
            out << "\u001b[36m";
        }

        // ASCII section
        for (uint16_t i = 0; i < 16; i++) {
            uint32_t k = line * 16 + i;
            if (k < len) {
                if (isprint((char)data[k]) && data[k] != 32) {
                    out << (char)data[k];
                } else {
                    //out << ".";
                    out << "\u00b7";
                }
            } else {
                out << " ";
            }
            if (i == 7) {
                out << " ";
            }
        }

        if (color) {   
            out << "\u001b[0m";
        }
        out << std::endl;
    }
}
