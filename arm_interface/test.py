#!/usr/bin/env python3
import serial
import time

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)


def write_read(x):
    arduino.write(bytes(x, 'ASCII'))
    time.sleep(.05)
    data = arduino.readline().decode('ASCII').rstrip()
    return data

if __name__=='__main__':
    while True:
        num = input("Enter a number: ")
        value = write_read(num)
        print(value)
