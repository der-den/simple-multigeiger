# Simple Geiger Counter

> **AI-Enhanced Project**: This project was improved with the assistance of Cascade AI, Used: Claude 3.7 Sonnet

A minimal ESP32-based Geiger counter application extracted from the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) project.

## Features

- High voltage generation for Geiger-Mueller tube
- Pulse counting with dead time filtering
- Calculation of CPM (counts per minute) and µSv/h radiation dose
- Serial output of radiation readings
- Audio feedback via speaker for each detected radiation pulse
- LED indication for radiation events

## Hardware Requirements

- ESP32 development board
- Geiger-Mueller tube
- High voltage generation circuit (as per MultiGeiger project)
- Speaker connected to PIN_SPEAKER_OUTPUT_P (GPIO 12)
- LED connected to built-in LED pin

## Pin Configuration

- PIN_HV_FET_OUTPUT (GPIO 23): High voltage FET control
- PIN_HV_CAP_FULL_INPUT (GPIO 22): Capacitor full detection
- PIN_GMC_COUNT_INPUT (GPIO 2): Geiger pulse detection
- PIN_SPEAKER_OUTPUT_P (GPIO 12): Speaker positive output
- PIN_SPEAKER_OUTPUT_N (GPIO 0): Speaker negative output

## Usage

1. Connect the hardware as specified in the pin configuration
2. Upload the sketch to your ESP32
3. Open the Serial Monitor at 115200 baud
4. The device will output radiation readings every second

## Configuration

You can customize the following parameters:
- `speaker_enabled`: Set to false to disable audio feedback
- `led_enabled`: Set to false to disable LED indication
- `TUBE_FACTOR`: Adjust based on your specific Geiger-Mueller tube
- `LOOP_DURATION`: Change the update frequency (default: 1000ms)

## Credits

This project is based on the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) project by ecocurious2.
