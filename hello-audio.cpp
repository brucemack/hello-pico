/*
Load command:
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program hello-audio.elf verify reset exit"

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
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h" // wait for interrupt 
 
// Audio PIN is to match some of the design guide shields. 
#define AUDIO_PIN 16  // you can change this to whatever you like

/* 
 * This include brings in static arrays which contain audio samples. 
 * if you want to know how to make these please see the python code
 * for converting audio samples into static arrays. 
 */
#include "ring.h"
int wav_position = 0;

/*
 * PWM Interrupt Handler which outputs PWM level and advances the 
 * current sample. 
 * 
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8   (this is what bitshifting <<3 is doing)
 * 
 */
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));    
    if (wav_position < (WAV_DATA_LENGTH<<3) - 1) { 
        // set pwm level 
        // allow the pwm value to repeat for 8 cycles this is >>3 
        pwm_set_gpio_level(AUDIO_PIN, WAV_DATA[wav_position>>3]);  
        wav_position++;
    } else {
        // reset to start
        wav_position = 0;
    }
}

int main(void) {
    /* Overclocking for fun but then also so the system clock is a 
     * multiple of typical audio sampling rates.
     */
    stdio_init_all();
    //set_sys_clock_khz(176000, true); 
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    // set the handle function above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate. 
     * 
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     * 
     * 
     * So clkdiv should be as follows for given sample rate
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    pwm_config_set_clkdiv(&config, 8.0f); 
    pwm_config_set_wrap(&config, 250); 
    pwm_init(audio_pin_slice, &config, true);

    pwm_set_gpio_level(AUDIO_PIN, 0);

    while(1) {
        __wfi(); // Wait for Interrupt
    }
}

/*
#define LED_PIN (25)
#define AUDIO_PIN (16)

using namespace std;

static const float PI = 3.1415926;

static const uint32_t pcmDataLen = 100;
static uint16_t pcmData[pcmDataLen];
static uint32_t outPtr = 0;
static uint32_t downsampleCount = 0;
static uint32_t downsampleLen = 8;

static void pwm_interrupt_handler() {

    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_PIN));        
    pwm_set_gpio_level(AUDIO_PIN, pcmData[outPtr]);  

    downsampleCount++;

    if (downsampleCount == downsampleLen) {
        downsampleCount = 0;

        outPtr++;
        if (outPtr == pcmDataLen) {
            outPtr = 0;
        }
    }
}

int main() {
 
    stdio_init_all();
    set_sys_clock_khz(176000, true); 

    // Stock LED setup
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // PWM setup
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);

    // Related to interrupt management
    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);

    // Setup PWM interrupt to fire when PWM cycle is complete
    pwm_clear_irq(audio_pin_slice);
    pwm_set_irq_enabled(audio_pin_slice, true);
    // Set the handle function above
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_interrupt_handler); 
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();

    // This sets the divider (8:4 format?)
    pwm_config_set_clkdiv(&config, 8.0); 
    // This sets the number of counts in the PMW cycle (i.e. "top" of 
    // the counter)
    pwm_config_set_wrap(&config, 250); 
    pwm_init(audio_pin_slice, &config, true);

    gpio_put(LED_PIN, 1);
    sleep_ms(1000);
    gpio_put(LED_PIN, 0);
    sleep_ms(1000);

    cout << "Hello, Audio Test" << endl;

    uint32_t p = 0;
    float phi = 0.0;
    float omega = 1500.0 * 2.0 * PI / 8000.0;
    for (uint32_t i = 0; i < 100; i++) {  
        float a = (std::cos(phi) + 1.0) / 2.0;
        pcmData[p] = 249.0 * a;
        cout << i << " " << a << " " << pcmData[p] << endl;
        phi += omega;
        p++;
    }


    while (1) {
    }
}

*/
