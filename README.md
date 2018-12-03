Firmware and Bootloader for the BuilderBot
==============

This repository contains the firmware and bootloader for the BuilderBot. The power board contains two microcontrollers, one for managing power (firmware-pm) and one for managing the sensors and actuators (firmware-sensact). In addition, there is a third microcontroller for controlling the manipulator (firmware-manip). This code uses AVR Libc and is based on the Arduino AVR core libraries. In many cases, the Arduino libraries have been significantly modified and only some parts of the code base still resemble the [original code](https://github.com/arduino/ArduinoCore-avr).

## Useful commands
1. Upload the bootloader
```bash
avrdude -c buspirate -p m328p -P /dev/ttyUSBX -U lock:w:0x3F:m -U lfuse:w:0xFF:m -U hfuse:w:0xDE:m -U efuse:w:0x05:m -U flash:w:optiboot_atmega328.hex -U lock:w:0x0F:m
```

2. Upload the firmware
```bash
avrdude -c arduino -p m328p -P /dev/ttyUSBX -b 57600 -U flash:w:firmware.hex
```

## Status LEDs

The following table summarizes the meaning of the LEDs on the BuilderBot powerboard.

| Battery LED | Charge LED | Meaning |
|-------------|------------|---------|
| ON          | ON         | Battery is present and fully charged |
| ON          | OFF        | Battery is present but not charging |
| ON          | BLINK      | Battery is present and charging |
| OFF         | ON         | Battery is not present, charger is ready  |
| OFF         | OFF        | Battery is not present, charger is in standby |
| OFF         | BLINK      | Battery is not present, charger has an error |
| BLINK       | ON         | Undefined fault |
| BLINK       | OFF        | Battery fault |
| BLINK       | BLINK      | Low battery |
