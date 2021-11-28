
print('importing libraries...')

import sys
import numpy as np
import enum

import arm.simulation.arm_model as arm_data
from arm.simulation.rectangular_to_angles import Converter

class ArmServo(enum.Enum):
    BASE_ROTATER = 6
    BOTTOM_JOINT = 5
    MIDDLE_JOINT = 4
    UPPER_JOINT  = 3
    CLAW_TWISTER = 2
    CLAW_CLOSER  = 1

class ArmController:
    """Class for controlling the arm directly."""

    def __init__(self, joint_distances=arm_data.joint_distances, precision=4,
                 serial_device="/dev/ttyUSB0"):
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

    def calculate_angles(self, coordinates, unit="inches"):
        """Determines the servo angles needed to move the arm to the specfied coordinates.
        Takes a triple of floats, and returns a dict containing the values needed to send 
        to the arm (in order)."""
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

        # rescale to servo inputs
        angles = [d*180/np.pi for d in thetas]
        servo_inputs = [int((a+90)/180*2000 + 500) for a in angles]

        return {
            ArmServo.BASE_ROTATER : servo_inputs[0],
            ArmServo.BOTTOM_JOINT : servo_inputs[1],
            ArmServo.MIDDLE_JOINT : servo_inputs[2],
            ArmServo.UPPER_JOINT : servo_inputs[3],
        }

    def connect_to_arm(self):
        """Connects to the arm Arduino via serial port."""
    
    def move_arm_to(self, arm_state):
        """Moves the arm to the given state."""
    
    def move_arm(self, arm_servo, offset):
        """Moves the specified arm servo by the given amount."""
    
    def close_claw(self):
        """Closes the arm's claw completely."""
    
    def open_claw(self):
        """Opens the arm's claw completely."""
