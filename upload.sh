#!/bin/bash

# Upload the sketch stored in the directory specified in $1 to the Arduino Uno.
echo "Compiling $1..."
arduino-cli compile -b arduino:avr:uno $1
echo "Done."
echo "Uploading $1..."
arduino-cli upload -b arduino:avr:uno -p /dev/ttyUSB0 $1
