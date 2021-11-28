
print('importing libraries...')

import sys
import numpy as np
import enum
import serial

import arm.simulation.arm_model as arm_data
from arm.simulation.rectangular_to_angles import Converter

class ArmServo(enum.IntEnum):
    BASE_ROTATER = 6
    BOTTOM_JOINT = 5
    MIDDLE_JOINT = 4
    UPPER_JOINT  = 3
    CLAW_TWISTER = 2
    CLAW_CLOSER  = 1

class ArmController:
    """Class for controlling the arm directly."""

    def __init__(self, joint_distances=arm_data.joint_distances, precision=4,
                 serial_device="/dev/ttyUSB0",serial_comms=True):
        self.converter = Converter(joint_distances, precision=precision)
        self.arm_state = {
            ArmServo.BASE_ROTATER : 1500,
            ArmServo.BOTTOM_JOINT : 1500,
            ArmServo.MIDDLE_JOINT : 1500,
            ArmServo.UPPER_JOINT : 1500,
            ArmServo.CLAW_TWISTER : 1500,
            ArmServo.CLAW_CLOSER : 1500
        }
        self.serial_device = serial_device
        self.baud_rate = 115200
        if serial_comms:
            self.arduino = serial.Serial(port=serial_device, baudrate=self.baud_rate, timeout=0.1)

    def calculate_servos(self, coordinates, unit="inches"):
        """Determines servo input values needed to move the arm to specified coordinates."""
        thetas = self.calculate_angles(coordinates, unit)
        return self.servo_angles(thetas)

    def servo_angles(self, thetas):
        """Rescales angles from calculate_angles() to servo input values."""
        # rescale to servo inputs
        print(thetas)
        angles = [d*180/np.pi for d in thetas]
        angles[1] = 90 - angles[1]
        angles[1] *= -1
        angles[3] *= -1
        print(angles)
        servo_inputs = [int((a+90)/180*2000 + 500) for a in angles]

        return {
            ArmServo.BASE_ROTATER : servo_inputs[0],
            ArmServo.BOTTOM_JOINT : servo_inputs[1],
            ArmServo.MIDDLE_JOINT : servo_inputs[2],
            ArmServo.UPPER_JOINT : servo_inputs[3],
        }

    def calculate_angles(self, coordinates, unit="inches"):
        """Determines the servo angles needed to move the arm to the specfied
        coordinates, in radians. Takes a triple of floats, and returns a list of
        floats."""
        # convert units
        if unit == "inches":
            point = np.array(coordinates)
        elif unit == "centimeters":
            point = np.array(c/2.54 for c in coordinates)
        elif unit == "legos":
            # definitely need to fix this calculation
            # we assume here that lego studs are 8mm apart, and the origin is aligned
            # with the arm
            point = np.array(c*8/25.4 for c in coordinates)
        
        # calculate angles
        try:
            thetas, _points, _theta, _phi = self.converter(point)
        except ValueError:
            print("Failed to calculate angles for given coordinate.")
            return None

        return thetas
    
    def move_to(self, position, unit="inches"):
        """Moves the arm to the given position, sending to arduino and updating
        internal arm state."""
        servo_positions = self.calculate_angles(position, unit)
        for servo in servo_positions:
            self.arm_state[servo] = servo_positions[servo]
        self.send_to_arduino()

    def send_to_arduino(self):
        """Sends current arm state to arduino over serial port."""
        # encode packet... this is kinda messy
        sorted_vals = sorted([(k,v) for k,v in self.arm_state.items()], key=lambda i: int(i[0]))
        packet = b'\x00'.join([bytes(str(i[1]), 'ASCII') for i in sorted_vals]) + b'\x00'
        self.arduino.write(packet)
        print(self.arduino.readline().decode('ASCII').rstrip())
    
    def close_claw(self):
        """Closes the arm's claw completely."""
    
    def open_claw(self):
        """Opens the arm's claw completely."""
