#include <stdio.h>
#include <cstring>
#include <iostream>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/sync.h"

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

static void set_reg8(spi_inst_t* spi, uint8_t block, uint16_t offset, uint8_t v) {
    uint8_t buf[4];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((block & 0b11111) << 3) | 0b1'00;
    // Data 
    buf[3] = v;
    spi_write_blocking(spi0, buf, 4);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void set_Sn_reg8(spi_inst_t* spi, uint8_t socket, uint16_t offset, uint8_t v) {
    set_reg8(spi, socket * 4 + 1, offset, v);
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

static void set_Sn_TXBUF(spi_inst_t* spi, uint8_t socket, uint16_t offset, 
    const uint8_t* buf2, uint16_t buf2Len) {
    // TODO: CLEAN UP AND DO THIS WITHOUT AN EXTRA COPY
    uint8_t buf[1024];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((socket * 4 + 2) << 3) | 0b1'00;
    // Data
    memcpy(buf + 3, buf2, buf2Len);
    spi_write_blocking(spi0, buf, 3 + buf2Len);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void set_MR(spi_inst_t* spi, uint8_t v) {
    set_reg8(spi, 0, 0, v);
}

static void set_SIR(spi_inst_t* spi, uint8_t v) {
    set_reg8(spi, 0, 0x17, v);
}

static void set_SIMR(spi_inst_t* spi, uint8_t v) {
    set_reg8(spi, 0, 0x18, v);
}

static void set_PHYCFGR(spi_inst_t* spi, uint8_t v) {
    set_reg8(spi, 0, 0x2e, v);
}

static void set_SHAR(spi_inst_t* spi, const uint8_t* mac) {
    uint8_t buf[3 + 6];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = 0;
    buf[1] = 9;
    // Control
    buf[2] = 0b00000 | 0b1'00;
    // Data 
    memcpy(buf + 3, mac, 6);
    spi_write_blocking(spi0, buf, 9);
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

static void set_Sn_TX_WR(spi_inst_t* spi, uint8_t socket, uint16_t v) {
    set_Sn_reg16(spi, socket, 0x24, v);
}

static void set_Sn_RX_RD(spi_inst_t* spi, uint8_t socket, uint16_t v) {
    set_Sn_reg16(spi, socket, 0x28, v);
}

static uint8_t get_reg8(spi_inst_t* spi, uint8_t block, uint16_t offset) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = (offset & 0xff00) >> 8;
    buf[1] = (offset & 0x00ff);
    // Control
    buf[2] = ((block & 0b11111) << 3) | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 1);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    return buf[0];
}    

static uint8_t get_Sn_reg8(spi_inst_t* spi, uint8_t socket, uint16_t offset) {
    return get_reg8(spi, socket * 4 + 1, offset);
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

static uint8_t get_MR(spi_inst_t* spi) {
    return get_reg8(spi, 0, 0);
}    

static uint8_t get_SIR(spi_inst_t* spi) {
    return get_reg8(spi, 0, 0x17);
}    

static uint8_t get_PHYCFGR(spi_inst_t* spi) {
    return get_reg8(spi, 0, 0x2e);
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

static uint8_t get_Sn_CR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg8(spi, socket, 0x01);
}    

static uint8_t get_Sn_IR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg8(spi, socket, 0x02);
}    

static uint8_t get_Sn_SR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg8(spi, socket, 0x03);
}    

static uint16_t get_Sn_TX_FSR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x20);
}    

static uint16_t get_Sn_TX_RD(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x23);
}    

static uint16_t get_Sn_TX_WR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x24);
}    

static uint16_t get_Sn_RX_RSR(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x26);
}    

static uint16_t get_Sn_RX_RD(spi_inst_t* spi, uint8_t socket) {
    return get_Sn_reg16(spi, socket, 0x28);
}    

// ====== PACKET STRUCTURE ===================================================

void formatMAC(const uint8_t* mac, char* buf) {
    snprintf(buf, 18, "%02x:%02x:%02x:%02x:%02x:%02x", 
        (int)mac[0], (int)mac[1], (int)mac[2], (int)mac[3], (int)mac[4], (int)mac[5]);
}

void formatIP4(const uint8_t* a, char* buf) {
    snprintf(buf, 16, "%d.%d.%d.%d", 
        (int)a[0], (int)a[1], (int)a[2], (int)a[3]);
}

uint16_t ntohs(uint16_t n) {
    return ((n & 0xff00) >> 8) | ((n & 0x00ff) << 8);
}

uint32_t htons(uint32_t n) {
    return ntohs(n);
}

uint32_t htonl(uint32_t n) {
    return ((n & 0xff000000) >> 24) | ((n & 0x00ff0000) >> 8) | 
        ((n & 0x0000ff00) << 8) | ((n & 0x000000ff) << 24);
}

uint32_t nltoh(uint32_t n) {
    return htonl(n);
}

uint16_t computeCs(uint8_t* data, uint32_t dataLen) {
    // Use a wider field to capture the carry
    uint32_t total = 0;
    for (uint32_t i = 0; i < dataLen; i += 2) {
        // Data is stored with MSB first
        uint16_t value = ((uint16_t)data[i] << 8) | data[i + 1];
        total += value;
    }
    uint16_t result = (uint16_t)total;
    
    // Re-apply the carry
    result += ((total >> 16) & 0xffff);

    return ~result;
}

#pragma pack(1)
struct EthHeader {

    uint8_t dstMAC[6];
    uint8_t srcMAC[6];
    uint16_t typ;

    bool isIP4() const { return ntohs(typ) == 0x0800; }
    
    bool isBroadcast() const { 
        return 
            dstMAC[0] == 0xff &&
            dstMAC[1] == 0xff &&
            dstMAC[2] == 0xff &&
            dstMAC[3] == 0xff &&
            dstMAC[4] == 0xff &&
            dstMAC[5] == 0xff;
    }

    bool isDstMAC(const uint8_t* mac) {
        return memcmp(mac, dstMAC, 6) == 0;
    }
};

#pragma pack(1)
struct ARP {
    uint16_t hwTyp;
    uint16_t protoTyp;
    uint8_t hwSz;
    uint8_t protoSz;
    uint16_t opcode;
    uint8_t sndMAC[6];
    uint8_t sndIP4[4];
    uint8_t tgtMAC[6];
    uint8_t tgtIP4[4];
};

#pragma pack(1)
struct IP4Header {
    // Ver 4, Hdr Len = 20
    uint8_t verHdrLen = 0x45;
    // Normally 0
    uint8_t dsf = 0;
    // Header (20) + Data Payload
    // Fill in after we know the full extent of the payload
    uint16_t totalLen;
    uint16_t id = 0;
    // Fragment related
    // 000. .... .... .... 010 Means don't fragment
    // ...0 0000 0000 0000 Fragment offset
    uint16_t flags = 0b000'0000000000000;
    uint8_t ttl = 128;
    uint8_t proto;
    uint16_t hdrCs = 0;
    uint8_t srcIP4[4];
    uint8_t dstIP4[4];

    bool isUDP() const {
        return proto == 17;
    }

    void populateHdrCs() {
        hdrCs = 0;
        hdrCs = htons(computeCs((uint8_t*)this, sizeof(IP4Header)));
    }
};

#pragma pack(1)
struct UDPHeader {

    uint16_t srcPort;
    uint16_t dstPort;
    // Header (8) + Data Payload
    // Fill in after we know the full extent of the payload
    uint16_t totalLen;
    uint16_t hdrCs = 0;

    void populateHdrCs() {
        hdrCs = 0;
        hdrCs = htons(computeCs((uint8_t*)this, sizeof(UDPHeader)));
    }

    uint16_t getDstPort() const { return ntohs(dstPort); }
};

#pragma pack(1)
struct DHCPHeader {
    uint8_t op;
    uint8_t htyp;
    uint8_t hlen;
    uint8_t hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint8_t cIP4[4];
    uint8_t yIP4[4];
    // Next
    uint8_t nIP4[4];
    // Relay
    uint8_t rIP4[4];
    uint8_t cMAC[6];
    // Padding for MAC
    char pad0[10];
    char pad1[192];
    uint32_t magic;
};

struct DHCPRequest {

    EthHeader eth;
    IP4Header ip;
    UDPHeader udp;
    DHCPHeader dhcp;

    // NOTE: We don't need to fill this whole thing
    // Uses standard <type><len><value ...> format.
    // Must be terminated by 0xff
    char options[256];
};

void setBroadcastMAC(uint8_t* mac) {
    mac[0] = 0xff;
    mac[1] = 0xff;
    mac[2] = 0xff;
    mac[3] = 0xff;
    mac[4] = 0xff;
    mac[5] = 0xff;
}

void setZeroIP4(uint8_t* a) {
    a[0] = 0;
    a[1] = 0;
    a[2] = 0;
    a[3] = 0;
}

void setBroadcastIP4(uint8_t* a) {
    a[0] = 0xff;
    a[1] = 0xff;
    a[2] = 0xff;
    a[3] = 0xff;
}

static const uint8_t eventMask = GPIO_IRQ_EDGE_FALL;

// For HW buffer (in W5500)
static const uint16_t blockSize = 16384;
static const uint16_t blockMask = blockSize - 1;

static const uint32_t rxBufferSize = 16384;
static const uint32_t rxBufferMask = rxBufferSize - 1;
uint8_t rxBuffer[rxBufferSize];
volatile uint32_t rxBufferRD = 0;
volatile uint32_t rxBufferWR = 0;
volatile uint32_t rxBufferOF = 0;

uint32_t rxBufferAvailable() {
    __dsb();
    return rxBufferWR - rxBufferRD;
}

void rxBufferRead(uint8_t* buf, uint32_t len) {
    if (len > rxBufferAvailable()) {
        panic("Not available");
    }
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = rxBuffer[rxBufferRD & rxBufferMask];
        rxBufferRD++;
        // TODO: WRAP CASE
    }
    __dsb();
}

void rxBufferPeek(uint8_t* buf, uint32_t len) {
    if (len > rxBufferAvailable()) {
        panic("Not available");
    }
    uint32_t tempRD = rxBufferRD;
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = rxBuffer[tempRD & rxBufferMask];
        tempRD++;
    }
    __dsb();
}

static void intHandler(uint gpio, uint32_t events) {

    // Now we need to clear the interrupt flags immediately to 
    // make sure we don't lose anything.
    //
    gpio_acknowledge_irq(W5500_INT_PIN, events);
    set_SIR(spi0, 0);

    // Capture event that triggered the interrupt
    uint8_t ir = get_Sn_IR(spi0, 0);
    // NOTE: We need to investigate the race-condition that exists
    // here when an interrupt happens on the S5500 chip between the 
    // time we read the IR and the time we clear the IR.  Can we
    // use an atomic read/write here?
    set_Sn_IR(spi0, 0, 0xff);

    if (ir & 0b001'0000) {
        cout << "Send OK" << endl;
    }

    if (ir & 0b000'0100) {

        // Pull data from the network as agressively as possible
        uint16_t rx_rsr = get_Sn_RX_RSR(spi0, 0);

        if (rx_rsr) {

            uint16_t rx_rd = get_Sn_RX_RD(spi0, 0);            
            // Compute the reads, possibly in two parts if we are 
            // close to the end of the W5500 buffer
            uint16_t offset0 = rx_rd & blockMask;
            uint16_t len0 = std::min(rx_rsr, (uint16_t)(blockSize - offset0));
            uint16_t offset1 = 0;
            uint16_t len1 = rx_rsr - len0;

            uint8_t buf[blockSize];

            get_Sn_RXBUF(spi0, 0, offset0, buf, len0);
            if (len1 > 0) {
                get_Sn_RXBUF(spi0, 0, offset1, buf + len0, len1);
            }

            // Move into the software buffer
            for (uint32_t i = 0; i < len0 + len1; i++) {
                rxBuffer[rxBufferWR & rxBufferMask] = buf[i];
                // Check for the overflow case
                if (((rxBufferWR + 1) & rxBufferMask) == (rxBufferRD & rxBufferMask)) {
                    rxBufferOF++;
                } else {
                    rxBufferWR++;
                    // TODO: WRAP CASE
                }
            }

            // Note that the management of RX_RD is independent of the 
            // size of the RX buffer.  The addressing just pretends
            // everything is 64K. This is expected to wrap at 0xffff!  
            rx_rd += rx_rsr;
            set_Sn_RX_RD(spi0, 0, rx_rd);
            // Tell the controller that we've done a receive
            set_Sn_CR(spi0, 0, 0x40);
        }
    }
}

bool sendMessage(spi_inst_t* spi, const uint8_t* d, uint16_t dLen, bool blocking) {

    uint16_t tx_fsr = get_Sn_TX_FSR(spi, 0);            
    uint16_t tx_wr = get_Sn_TX_WR(spi, 0);            

    if (dLen > tx_fsr) {
        return false;
    }
    else {
        // It's possible that we'd need to do this in two parts
        // because of buffer wrap
        uint16_t offset0 = tx_wr;
        uint16_t len0 = std::min(dLen, (uint16_t)(blockSize - offset0));
        uint16_t offset1 = 0;
        uint16_t len1 = dLen - len0;
        set_Sn_TXBUF(spi, 0, offset0, d, len0);
        if (len1 > 0) {
            set_Sn_TXBUF(spi, 0, offset1, d + len0, len1);
        }
        // Adjust pointers.  No regard to wrapping in this logic
        set_Sn_TX_WR(spi, 0, tx_wr + (len0 + len1));
        // Tell the controller that we've done a SEND
        set_Sn_CR(spi, 0, 0x20);
        // Optionally wait for the SEND command to be executed
        if (blocking) {
            // TODO: TIMEOUT
            while (get_Sn_CR(spi0, 0));
        }
        return true;
    }
}

void initW5500(spi_inst_t* spi, const uint8_t* myMAC) {

    // Do the formal reset
    gpio_put(W5500_RST_PIN, 0);
    //sleep_ms(1);
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_RST_PIN, 1);

    // Long wait 
    sleep_ms(100);

    // Reset, wait until done
    set_MR(spi, get_MR(spi) | 0x80);
    while (get_MR(spi) != 0);

    sleep_ms(250);
    
    // Set MAC address
    set_SHAR(spi, myMAC);

    // PHY RESET, wait until done
    set_PHYCFGR(spi, get_PHYCFGR(spi0) | 0x80);
    while ((get_PHYCFGR(spi) & 0x80) == 0);
    // 100/Full/Manual seems to work
    set_PHYCFGR(spi, 0xd8);
    // Wait for link to come up
    while (get_PHYCFGR(spi) != 0xdf);

    sleep_ms(250);

    // Adjust buffer size up to maximum
    set_Sn_RXBUF_SIZE(spi, 0, 16);
    set_Sn_TXBUF_SIZE(spi, 0, 16);
    // S0_MR Setup (MACRAW)
    // MAC filter enabled
    set_Sn_MR(spi, 0, 0b1001'0100);
    // Enable socket 0 interrupts
    set_SIMR(spi, 1);

    // S0_CR Setup (OPEN)
    set_Sn_CR(spi, 0, 0x01);
    // Wait for status = OPEN
    while (get_Sn_SR(spi, 0) != 0x42);

    // Clear interrupts
    set_SIR(spi, 0);
    set_Sn_IR(spi, 0, 0xff);
    gpio_set_irq_enabled_with_callback(W5500_INT_PIN, eventMask, true, intHandler);
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

    // Just made this up
    uint8_t myMAC[6] = { 0xd4, 0xbe, 0xd9, 0xe8, 0xba, 0xbe };
    initW5500(spi0, myMAC);

    DHCPRequest req;
    uint16_t reqLen;

    // Make a DHCP (DISCOVER) request
    {        
        // Ethernet 
        setBroadcastMAC(req.eth.dstMAC);
        memcpy(req.eth.srcMAC, myMAC, 6);
        req.eth.typ = htons(0x0800);
        
        // IP4
        req.ip.proto = 17;
        req.ip.totalLen = htons(sizeof(DHCPRequest) - sizeof(EthHeader));
        setZeroIP4(req.ip.srcIP4);
        setBroadcastIP4(req.ip.dstIP4);

        // UDP
        req.udp.srcPort = htons(68);
        req.udp.dstPort = htons(67);

        // DHCP
        req.dhcp.op = 1;
        req.dhcp.htyp = 1;
        req.dhcp.hlen = 6;
        req.dhcp.hops = 0;
        req.dhcp.xid = 7;
        req.dhcp.secs = 0;
        // Turn on broadcast?
        req.dhcp.flags = htons(0x8000);
        // Unicast - this causes the offer to be sent directly
        // to us instead of being broadcast
        //req.dhcp.flags = htons(0x0000);
        setZeroIP4(req.dhcp.cIP4);
        setZeroIP4(req.dhcp.yIP4);
        // Next
        setZeroIP4(req.dhcp.nIP4);
        // Relay
        setZeroIP4(req.dhcp.rIP4);
        memcpy(req.dhcp.cMAC, myMAC, 6);
        memset(req.dhcp.pad0, 0, 10);
        memset(req.dhcp.pad1, 0, 192);
        req.dhcp.magic = htonl(0x63825363);

        // 9 bytes of options used
        req.options[0] = 0x35;
        req.options[1] = 0x01;
        req.options[2] = 0x01;

        // Client identifier
        req.options[3] = 0x3d;
        req.options[4] = 7;
        req.options[5] = 1;
        memcpy(&(req.options[6]), myMAC, 6);

        // Requests
        req.options[12] = 0x37;
        req.options[13] = 0x03;
        req.options[14] = 0x01;
        req.options[15] = 0x03;
        req.options[16] = 0x06; 
   
        req.options[17] = 0xff;

        // Fill in lengths
        reqLen = sizeof(DHCPRequest) - 256 + 18;
        req.ip.totalLen = htons(reqLen - sizeof(EthHeader));
        req.udp.totalLen = htons(reqLen - sizeof(EthHeader) - sizeof(IP4Header));
        // Do this last!
        // Optional?
        //req.udp.populateHdrCs();
        req.ip.populateHdrCs();
    }

    // Transmit the packet
    sendMessage(spi0, (const uint8_t*)&req, reqLen, false);

    // Receive loop
    int block = 16;

    while (block > 0) {     

        // Check to see if we have at least a 2-byte length word available
        if (rxBufferAvailable() < 2) {
            continue;
        }
        uint8_t lenBuf[2];
        rxBufferPeek(lenBuf, 2);
        // This length includes the length bytes
        uint16_t len = ((uint16_t)lenBuf[0] << 8) | lenBuf[1];
        // Check to see if we can pull in the full message
        if (rxBufferAvailable() < len) {
            continue;
        }

        // Pull out a complete message
        uint8_t buf[blockSize] __attribute__ ((aligned(32)));
        // (Discard length)
        rxBufferRead(lenBuf, 2);
        len -= 2;
        rxBufferRead(buf, len);

        // Process packet
        auto ethHdr = (const EthHeader*)buf;
        if (ethHdr->isIP4()) {
            cout << "A" << endl;
            auto ipHdr = (const IP4Header*)(buf + sizeof(EthHeader));
            if (ipHdr->isUDP()) {
                cout << "B" << endl;
                auto udpHdr = (const UDPHeader*)(buf + sizeof(EthHeader) + sizeof(IP4Header));
                if (udpHdr->getDstPort() == 68) {

                    char buf2[20];
                    formatMAC(ethHdr->dstMAC, buf2);
                    cout << "DST: " << buf2 << endl;
                    formatMAC(ethHdr->srcMAC, buf2);
                    cout << "SRC: " << buf2 << endl;

                    cout << "--------------------------------------------------" << endl;
                    prettyHexDump(buf, len, cout, true);
                }
            }
        }

        block--;
    }

    cout << "OF " << rxBufferOF << endl;
    cout << "RSR " << get_Sn_RX_RSR(spi0, 0) << endl;
    cout << "Done" << endl;

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
