/*
Build
    
    cd build
    cmake ..
    make hello-async

Load command:
    
    openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program hello-async.elf verify reset exit"

Minicom (for console, not the UART being tested):
    
    minicom -b 115200 -o -D /dev/ttyACM0
*/
#include <cstdint>
#include <iostream>
#include <cmath>

#include "pico.h"
#include "pico/stdlib.h"
#include "pico/time.h"
//#include "pico/async_context.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#define LED_PIN (25)

using namespace std;

//static void do_work(async_context_t *context, struct async_work_on_timeout *timeout) {    
//    cout << "Tick" << endl;
//}

int main(int,const char**) {

    stdio_init_all();

    // Stock LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    cout << "Hello Async" << endl;

    //async_context_poll_t ctx;
    //async_context_poll_init_with_defaults(&ctx);

    //absolute_time_t now = get_absolute_time();
    //absolute_time_t target = delayed_by_us(now, 1000000);

    //async_at_time_worker_t worker;
    //worker.next_time = target;
    //worker.do = do_work;

	//async_context_add_at_time_worker(&ctx, &worker);

    cout << "Start ..." << endl;

    while (true) {
        
        //async_context_poll(&ctx);

    }

    return 0;
}
