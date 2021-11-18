#!/usr/bin/env python3
import serial
import time

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)


def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data

if __name__=='__main__':
    while True:
        num = input("Enter a number: ")
        value = write_read(num)
        print(value)