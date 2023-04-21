#!/bin/bash

# Check argument
if ! [ -f "/home/tschmitz/catkin_ws/src/startup/$1" ]; then
	exit
fi

if ! [ -d /home/tschmitz/catkin_ws/src/firmware/distance_sensors/_build ]; then
	mkdir /home/tschmitz/catkin_ws/src/firmware/distance_sensors/_build
fi

if ! [ -d /home/tschmitz/catkin_ws/src/firmware/horizontal_movement/_build ]; then
	mkdir /home/tschmitz/catkin_ws/src/firmware/horizontal_movement/_build
fi

movement_port=$(map-ports motors)
sensors_port=$(map-ports sensors)

# Build and upload arduino code 
/home/tschmitz/Downloads/arduino-1.8.19/arduino-builder -compile -logger=machine -hardware /home/tschmitz/Downloads/arduino-1.8.19/hardware -tools /home/tschmitz/Downloads/arduino-1.8.19/tools-builder -tools /home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -built-in-libraries /home/tschmitz/Downloads/arduino-1.8.19/libraries -libraries /home/tschmitz/Arduino/libraries -fqbn=arduino:avr:mega:cpu=atmega2560 -ide-version=10819 -build-path /home/tschmitz/catkin_ws/src/firmware/horizontal_movement/_build -warnings=none -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino7.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avrdude.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.arduinoOTA-1.3.0.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -verbose /home/tschmitz/catkin_ws/src/firmware/horizontal_movement/horizontal_movement.ino
/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr/bin/avrdude -C/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr/etc/avrdude.conf -v -patmega2560 -cwiring -P$(movement_port) -b115200 -D -Uflash:w:/home/tschmitz/catkin_ws/src/firmware/horizontal_movement/_build/horizontal_movement.ino.hex:i 
/home/tschmitz/Downloads/arduino-1.8.19/arduino-builder -compile -logger=machine -hardware /home/tschmitz/Downloads/arduino-1.8.19/hardware -tools /home/tschmitz/Downloads/arduino-1.8.19/tools-builder -tools /home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -built-in-libraries /home/tschmitz/Downloads/arduino-1.8.19/libraries -libraries /home/tschmitz/Arduino/libraries -fqbn=arduino:avr:mega:cpu=atmega2560 -ide-version=10819 -build-path /home/tschmitz/catkin_ws/src/firmware/distance_sensors/_build -warnings=none -prefs=build.warn_data_percentage=75 -prefs=runtime.tools.avr-gcc.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino7.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avrdude.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.arduinoOTA.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -prefs=runtime.tools.arduinoOTA-1.3.0.path=/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr -verbose /home/tschmitz/catkin_ws/src/firmware/distance_sensors/distance_sensors.ino
/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr/bin/avrdude -C/home/tschmitz/Downloads/arduino-1.8.19/hardware/tools/avr/etc/avrdude.conf -v -patmega2560 -cwiring -P$(sensors_port) -b115200 -D -Uflash:w:/home/tschmitz/catkin_ws/src/firmware/distance_sensors/_build/distance_sensors.ino.hex:i

# Launch ros
roslaunch startup "$1"

