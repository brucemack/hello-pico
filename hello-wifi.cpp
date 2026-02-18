/*
Building:

    export PICO_SDK_FETCH_FROM_GIT=1
    mkdir build
    cd build
    cmake .. -DPICO_BOARD=pico_w
    make hello-wifi

Flashing:

    ~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program hello-wifi.elf verify reset exit"

Serial console:

    minicom -b 115200 -o -D /dev/ttyACM1
*/

#include <stdio.h>
#include <iostream>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define LED_PIN (25)

using namespace std;

char ssid[] = "Gloucester Island Municipal WIFI";
char pass[] = "xxx";
 
static void myRecv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    printf("Got %c\n", pbuf_get_at(p, 0));
    pbuf_free(p);
}

int main() {
    
    stdio_init_all();

    // Stock LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    cyw43_arch_wifi_connect_async(ssid, pass, CYW43_AUTH_WPA2_AES_PSK);

    cout << "Starting ..." << endl;
    bool isUp = false;    
    struct udp_pcb* u = 0;

    while (true) {
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        
        //if (cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_JOIN);
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP) {
            if (!isUp) {
                cout << "Good" << endl;
                cout << ipaddr_ntoa(&cyw43_state.netif[0].ip_addr) << endl;
                isUp = true;
    
                // Setup a UDP port
                u = udp_new_ip_type(IPADDR_TYPE_ANY);
                if (!u) {
                    cout << "Failed to create pcb" << endl;
                    return 0;
                }
                udp_bind(u, IP_ADDR_ANY, 5190);

                // Install a receive
                udp_recv(u, myRecv, 0);
            }

        }
    }


}

