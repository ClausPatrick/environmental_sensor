# Raspberry Pi Pico RP2040 Multi-Sensor Environmental Monitoring System with U2U messaging protocol

## Project Overview

This embedded systems project is a comprehensive environmental monitoring solution that integrates multiple sensors to collect and process environmental data. The system leverages I2C communication to read data from various sensors and display information on an LCD screen.

## Key Features


### Color Gradient
The RGB LED provides a dynamic, intuitive visualization of air quality:
- <b>Blue to Green to Red Gradient</b>: Smooth color transition representing escalating air quality concerns
  - Blue: Good air quality
  - Green: Moderate air quality
  - Red: Poor air quality

### Alert Stages
1. <b>First Threshold Exceeded</b>:
   - LED alternates between <b>Yellow and Red</b>
   - Indicates significant air quality degradation

2. <b>Second Threshold Exceeded</b>:
   - LED flashes between <b>Red and Blue</b>
   - Signifies critical air quality conditions

### Visualization Characteristics
- <b>Seamless Color Transitions</b>: Uses PWM for smooth color blending
- <b>Intuitive Color Mapping</b>: Instantly communicable air quality status
- <b>Progressive Warning System</b>: Increasing visual urgency with air quality decline

- **Sensor Integration**:
  - <b>SGP40 VOC (Volatile Organic Compounds) Sensor</b>
  - <b>AM2320 Temperature and Humidity Sensor</b>
  - <b>PMSA003i Particulate Matter Sensor</b>

- **Data Processing**:
  - Real-time sensor data collection
  - VOC index calculation using Gas Index Algorithm
  - Particle size measurement
  - CRC error checking

- **Display**:
  - LCD display of environmental parameters and custom text via U2U
  - RGB LED indication based on VOC levels

- **Text display**:
  - If the length of the text received exceeds the 32 character limit it will implement a 'scrolling' behaviour: Text will roll over the two lines, its speed can be configured by changing `SCROLL_DELAY`. 

- **Multicore Architecture**:
  - Utilizes dual-core processing
  - Separate cores for sensor reading and LCD display

- **U2U Messaging**:
  - Sensor data can be requested via U2U on PORT0 or PORT1 (see configuration). The unit responds to Topic `GET_SNSR` when addressed by its node name or by `GEN`. The message Payload will contain following quantities:
    - temperature 
    - humidity
    - volitile organic compound 
    - particle density data (14 values)
  - Two lines of the 16*4 are free for custom text. The unit will transfer payload it received on messages with Topic `SET_LCD_` to the lower two lines. 

## Hardware Requirements

- Microcontroller (Raspberry Pi Pico or similar)
- I2C Sensors:
  - SGP40 (VOC)
  - AM2320 (Temperature/Humidity)
  - PMSA003i (Particulate Matter)
- LCD Display (4x16 character)
- RGB LED

## Software Dependencies

- Sensirion I2C HAL
- Sensirion Gas Index Algorithm Library
- Raylib (if applicable)


## Configuration

Edit the main source file to adjust:
- GPIO pin for LED air quality visualisation: `RGB_G_PIN`, `RGB_B_PIN`, `RGB_R_PIN`
- `NUM_PIXELS` Number of LEDs in the strip
- LCD control and data pins: `RS` `E`, `DB4`, `DB5`, `DB6`, `DB7`, `RS_COMMAND`, `RS_DATA`
- I2C data and clock pin: `SCL_PIN`, `SDA_PIN`
- I2C sensor address: `PMSA_ADDR`, `SGP_ADDR`, `AM_ADDR`, if needed.
- The module uses a set of LEDs to indicate various statuses. If omitted the `output_nr` needs changing

Edit `u2u_client_profile` to configure:
- `SELF_NAME` node's name
- `u2u_platform_channels 2` To allow for communication via UART0 and UART1
- `UART_RX_IN`  GPIO pin number to receive on PORT0
- `UART_TX_IN`  GPIO pin number to send on PORT0
- `UART_RX_OUT` GPIO pin number to receive on PORT1
- `UART_TX_OUT` GPIO pin number to send on PORT1


## Build and Upload

1. Install the [Pico SDK](https://github.com/raspberrypi/pico-sdk)
2. Clone or download this repository and create project directory.
3. Install openocd via package manager:
   ```bash
   cd ~/pico
   sudo apt install automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0- dev
   git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
   cd openocd
   ./bootstrap
   ./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio
   make -j4
   sudo make install
   ```
4. Create a `build` directory:
   ```bash
   mkdir build && cd build
   export PICO_SDK_PATH=../../pico-sdk
   cmake ..
   make
   ```
5. Upload elf file into target from within the buid directory:
   ```bash
   openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c program  project_name.elf verify reset exit
   ```


## Sensor Data Collection

The system collects the following environmental parameters:
- Temperature (°C)
- Humidity (%)
- Volatile Organic Compounds (VOC Index)
- Particulate Matter concentrations

### Sampling Characteristics
- <b>Sampling Interval</b>: Configurable (default 1 second)
- <b>Error Handling</b>: CRC validation, I2C error reporting

## Multicore Architecture

### Core 0 (Main Core)
- Sensor data reading
- Message processing
- Global state management

### Core 1 (Display Core)
- LCD display update
- Continuous environmental data visualization

## LED Indicators

The RGB LED provides visual feedback:
- Color intensity based on VOC levels
- Blinking patterns for different system states

## Error Handling

- Comprehensive I2C error reporting
- GPIO-based error signaling
- Graceful error recovery mechanisms

## Future Improvements

- Add more sensor types
- Implement data logging
- Enhance visualization techniques
- Create wireless data transmission module

## Compilation and Setup

1. Install required toolchains
2. Configure sensor addresses
3. Set up I2C and GPIO configurations
4. Compile and flash to microcontroller


[Your Name]

