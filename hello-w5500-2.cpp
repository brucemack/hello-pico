#include <stdio.h>
#include <cstring>
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

static void set_Sn_TXBUF(spi_inst_t* spi, uint8_t socket, uint16_t offset, 
    const uint8_t* buf2, uint16_t buf2Len) {
    // TODO: CLEAN UP!
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
    uint8_t buf[4];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = 0;
    buf[1] = 0;
    // Control
    buf[2] = 0b00000 | 0b1'00;
    // Data 
    buf[3] = v;
    spi_write_blocking(spi0, buf, 4);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void set_PHYCFGR(spi_inst_t* spi, uint8_t v) {
    uint8_t buf[4];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = 0;
    buf[1] = 0x2e;
    // Control
    buf[2] = 0b00000 | 0b1'00;
    // Data 
    buf[3] = v;
    spi_write_blocking(spi0, buf, 4);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
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

static uint8_t get_MR(spi_inst_t* spi) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = 0;
    buf[1] = 0;
    // Control
    buf[2] = 0b00000 | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 1);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    return buf[0];
}    

static uint8_t get_PHYCFGR(spi_inst_t* spi) {
    uint8_t buf[3];
    // S0_CR Setup
    gpio_put(SPI0_CSN_PIN, 0);
    asm volatile("nop \n nop \n nop");
    // Offset
    buf[0] = 0;
    buf[1] = 0x2e;
    // Control
    buf[2] = 0b00000 | 0b0'00;
    spi_write_blocking(spi0, buf, 3);
    spi_read_blocking(spi0, 0, buf, 1);
    gpio_put(SPI0_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
    return buf[0];
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

struct Ethernet2Header {
    uint8_t dstMAC[6];
    uint8_t srcMAC[6];
    uint16_t typ;
};

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

    Ethernet2Header eth;
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

int main() {
 
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
       
    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    cout << "Hello, W5500 test" << endl;

    // From: https://en.wikipedia.org/wiki/Internet_checksum#Computation
    uint8_t t[18];
    t[0] = 0x45;
    t[1] = 0x00;
    t[2] = 0x00;
    t[3] = 0x73;
    t[4] = 0x00;
    t[5] = 0x00;
    t[6] = 0x40;
    t[7] = 0x00;
    t[8] = 0x40;
    t[9] = 0x11;
    t[10] = 0xc0;
    t[11] = 0xa8;
    t[12] = 0x00;
    t[13] = 0x01;
    t[14] = 0xc0;
    t[15] = 0xa8;
    t[16] = 0x00;
    t[17] = 0xc7;
    printf("%x\n", computeCs(t, 18));

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
    sleep_ms(1);
    asm volatile("nop \n nop \n nop");
    gpio_put(W5500_RST_PIN, 1);

    // Long wait 
    sleep_ms(560);

    //uint8_t myMAC[6] = { 0x45, 0x01, 0x01, 0x00, 0xba, 0xbe};
    uint8_t myMAC[6] = { 0xd4, 0xbe, 0xd9, 0xe8, 0xca, 0xca };

    // Reset
    set_MR(spi0, get_MR(spi0) | 0x80);
    while (true) {
        if (get_MR(spi0) == 0) {
            break;
        }
        cout << "Wait" << endl;
        sleep_ms(10);
    }
    
    // Set address
    set_SHAR(spi0, myMAC);

    // PHY RESET
    set_PHYCFGR(spi0, get_PHYCFGR(spi0) | 0x80);
    while ((get_PHYCFGR(spi0) & 0x80) == 0);
    // 100/Full/Manual
    set_PHYCFGR(spi0, 0xd8);
    sleep_ms(5 * 1000);
    printf("PHYCFGR %x\n", (int)get_PHYCFGR(spi0));

    // Adjust buffer size up to maximum
    set_Sn_RXBUF_SIZE(spi0, 0, 16);
    set_Sn_TXBUF_SIZE(spi0, 0, 16);
    // S0_MR Setup (MACRAW)
    set_Sn_MR(spi0, 0, 0b1001'0100);
    // Turn off MAC filter
    //set_Sn_MR(spi0, 0, 0b0001'0100);
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
        req.ip.totalLen = htons(sizeof(DHCPRequest) - sizeof(Ethernet2Header));
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
        cout << "IP4 Total Len " << reqLen << endl;
        req.ip.totalLen = htons(reqLen - sizeof(Ethernet2Header));
        req.udp.totalLen = htons(reqLen - sizeof(Ethernet2Header) - sizeof(IP4Header));
        // Do this last!
        // Optional?
        //req.udp.populateHdrCs();
        req.ip.populateHdrCs();
    }

    const uint16_t blockSize = 16384;
    const uint16_t blockMask = blockSize - 1;

    // Transmit the DHCP packet
    {
        uint16_t tx_fsr = get_Sn_TX_FSR(spi0, 0);
        cout << "TX_FSR " << tx_fsr << endl;
        uint16_t tx_wr = get_Sn_TX_WR(spi0, 0);            
        cout << "TX_WR " << tx_wr << endl;
        uint16_t tx_rd = get_Sn_TX_RD(spi0, 0);            
        cout << "TX_RD " << tx_rd << endl;
        uint16_t offset0 = 0;
        uint16_t len0 = reqLen;
        uint16_t len1 = 0;
        // Write the data
        set_Sn_TXBUF(spi0, 0, offset0, (const uint8_t*)&req, len0);
        // Adjust pointers
        tx_wr += (len0 + len1);
        cout << "Writing new TX_WR " << tx_wr << endl;
        set_Sn_TX_WR(spi0, 0, tx_wr);
        // Tell the controller that we've done a send
        set_Sn_CR(spi0, 0, 0x20);
        // Wait for the command to be executed
        while (get_Sn_CR(spi0, 0));
    
        prettyHexDump((const uint8_t*)&req, len0, cout, true);

        tx_fsr = get_Sn_TX_FSR(spi0, 0);
        cout << "TX_FSR " << tx_fsr << endl;
        tx_wr = get_Sn_TX_WR(spi0, 0);            
        cout << "TX_WR " << tx_wr << endl;
        tx_rd = get_Sn_TX_RD(spi0, 0);            
        cout << "TX_RD " << tx_rd << endl;
    }

    // Receive loop
    int blocks = 16;

    while (1) {     

        uint8_t ir = get_Sn_IR(spi0, 0);
        set_Sn_IR(spi0, 0, 0xff);

        if (ir & 0x04) {
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

            const uint8_t* workPtr = buf;
            uint16_t workLen = len0 + len1;
            int part = 0;

            while (workLen > 0) {

                const uint16_t len = ((uint16_t)workPtr[0] << 8) | workPtr[1];
                const Ethernet2Header* ethHdr = (const Ethernet2Header*)(workPtr + 2);

                if (len > workLen) {
                    cout << "LENGTH WRONG" << endl;
                    workLen = 0;
                }
                else {

                    //cout << "TYP: " << ntohs(ethHdr->typ) << endl;
                    /*

                    if (ntohs(ethHdr->typ) == 0x0806) {
                        const ARP* arp = (const ARP*)(buf + 2 + sizeof(Ethernet2Header));
                        cout << "ARP op=" << ntohs(arp->opcode) << endl;
                        formatMAC(arp->sndMAC, buf2);
                        cout << "SND MAC: " << buf2 << endl;
                        formatIP4(arp->sndIP4, buf2);
                        cout << "SND IP4: " << buf2 << endl;
                        formatMAC(arp->tgtMAC, buf2);
                        cout << "TGT MAC: " << buf2 << endl;
                        formatIP4(arp->tgtIP4, buf2);
                        cout << "TGT IP4: " << buf2 << endl;
                    }
                    */

                    if (*(workPtr + 2 + 35) == 0x43 ||
                        *(workPtr + 2 + 35) == 0x44) {

                        char buf2[20];
                        formatMAC(ethHdr->dstMAC, buf2);
                        cout << "DST: " << buf2 << endl;
                        formatMAC(ethHdr->srcMAC, buf2);
                        cout << "SRC: " << buf2 << endl;

                        cout << "--------------------------------------------------" << endl;
                        prettyHexDump(workPtr + 2, len - 2, cout, true);
                    }

                    workPtr += len;
                    workLen -= len;
                } 
            }

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

        sleep_ms(500);
    }

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
