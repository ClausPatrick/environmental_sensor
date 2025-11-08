# Multi-Sensor Environmental Monitoring System

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
  - LCD display of environmental parameters
  - RGB LED indication based on VOC levels

- **Multicore Architecture**:
  - Utilizes dual-core processing
  - Separate cores for sensor reading and LCD display

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
