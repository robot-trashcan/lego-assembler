#!/bin/bash

# Upload the sketch stored in the directory specified in $1 to the Arduino Uno.
arduino-cli compile -b arduino:avr:uno $1
arduino-cli upload -b arduino:avr:uno -p /dev/ttyUSB0 $1
