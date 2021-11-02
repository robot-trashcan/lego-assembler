#!/bin/bash

arduino-cli compile -b arduino:avr:uno $1
arduino-cli upload -b arduino:avr:uno -p /dev/ttyUSB0 $1
