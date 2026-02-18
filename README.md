
Setup Steps for New Pi5/Pico2 
============================= 

I started with a clean Pi5, Pico2, and Pico Debug Probe.  These
are the steps needed to make it work.

Pi5 Setup

* Make a SD card with the latest Raspberry Pi OS.  
  * Make sure that SSH is enabled and configure the WiFi SSID/password.
  * Add enable_uart=1 to boot.txt
  * Add dtparam=uart0 and dtparam=uart0_console. This enables uart0, maps
it to GPIO14/15, and makes it the console.
* Do the same thing with the 1TB SSD drive.
* Insert the SD card and boot the Pi5.  Connect to the serial console.
* Log in and use sudo raspi-config to change the boot order. Advanced -> Boot Order -> USB Boot.
* sudo shutdown now
* Remove the SD card and connect the SSD using the blue USB port (high speed).
* Boot the Pi5.  Connect to the serial console.
* sudo apt update; sudo apt full-upgrade
* sudo raspi-config, enable VNC.

Setup Debug Probe

* Download debug probe firmware UF2 from RPi website.
* Connect USB with BOOTSEL button pressed.
* Drag the UF2 to the upload folder on the probe.
* Install the JST plug ("U")

Setup Pi5 for Pi Pico2 Development

* Uinstall the standard openocd: sudo apt remote openocd
* Install build tools

        sudo apt install python3 git tar build-essential
        sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

* Clone openocd from the RPi GIT repo (branch), build.
* Clone the pico-sdk from the RPi GIT repo.
* cd pico-sdk; git submodule update --init 
* set PICO_SDK_PATH

Build/Debug Commands
====================

One-time setup of make process:

        mkdir build
        cd build
        cmake -DPICO_BOARD=pico2 ..

Command used to flash code to board:        

        ~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "rp2350.dap.core1 cortex_m reset_config sysresetreq" -c "program main.elf verify reset exit"

Steps to Build CMSIS-DSP
========================

        cd ~/pico
        mkdir CMSISDSP
        cd CMSISDSP
        git clone https://github.com/ARM-software/CMSIS-DSP.git        
        git clone https://github.com/ARM-software/CMSIS_6.git
        cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
        (Setup the CMakeLists.txt shown below)
        mkdir build
        cd build
        cmake ..
        make -j4

What the CMakeLists.txt looks like:

```
cmake_minimum_required (VERSION 3.6)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake) 

set(HOME $ENV{HOME})
set(CMSISDSP ${HOME}/pico/CMSISDSP/CMSIS-DSP)
set(CMSISCORE ${HOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core)

# Define the project
project (cmsis-dsp VERSION 0.1)

# Initialise the Pico SDK
pico_sdk_init()

add_subdirectory(${CMSISDSP}/Source bin_dsp)

target_compile_options(CMSISDSP PUBLIC 
    -Wsign-compare
    -Wdouble-promotion
    -Ofast -ffast-math
    -DNDEBUG
    -Wall -Wextra  -Werror
    -fshort-enums 
    #-fshort-wchar
)
```


