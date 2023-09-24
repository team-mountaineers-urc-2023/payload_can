#!/usr/bin/env python3

import numpy as np
import can
import rospy
from can_master.msg import can_in, can_out

# Command Types
DC = 0
GPIO = 1
SERVO = 2
SOLENOID = 3

PICO_ID = 0x201
COMPUTER_ID = 0x200

CAN_TIMEOUT = 0.3

class Payload:
    """
    Payload Class
        Used to write and read information through the Payload PCB for 
        GPIO, DC, Servo, and Stepper Motors
    ------------------
    Supported Types 
    ------------------
    - "GPIO"
    - "DC"
    - "SERVO"
    - "STEPPER" Not really but if needed can be enabled
    ----------
    Functions
    ----------
    - GPIO_write(id, value)
    - GPIO_update()
    - DC_write(id, dir, data)
    - DC_HIGH_CURR_write(id, dir, data)
    - SERVO_write(id, data)    
    - PUMP_write(self, id, data)


    """

    def __init__(self):
        self.pub = rospy.Publisher("can_out", can_out, queue_size=10)
        self.sub = rospy.Subscriber("can_in", can_in, self.response_cb)
        self.expect_response = False
        self.response = None
        self.GPIO = None


    def __write(self, type:int, id:int, mod:int, data:int):
        """
        Converts information into valid format for CAN Payload Message
        then writes to motor on the CAN bus
        
        """
        msg = [0, 0, 0]
        msg[0] = (type << 6) | (id << 1) | mod  
        msg[1] = data >> 8      # High Byte of data
        msg[2] = data & 0xFF    # Low Byte of data
        
        out_msg = can_out() # build
        out_msg.arb_id = PICO_ID 
        out_msg.sub_id = msg[0]
        out_msg.data = msg

        self.pub.publish(out_msg)
            
    def response_cb(self, msg: can_in):
        """
        Callback that gets mounted to can_in topic
        checks arb_id to filter out 
        """
        if not self.expect_response or msg.arb_id != COMPUTER_ID:
            return
        self.GPIO = [False for i in range(8)]
        # check if each is receiving a signal
        # pins are default high and pulled down
        # flip them here so 1 is False and 0 is True
        for i in range(8):
            self.GPIO[i] = ((msg.data[0] >> i) & 0x01) == 0
        self.expect_response = False

    def solenoid_write(self, id: int, enable: bool):
        """
        Writes data to specified GPIO pin
        
        Params
        -------
        - id : int
            - Range (0-3)
            - ID of the motor driver to use
        - value : bool
            - True or False
            - enables or disables solenoid
        
        """
        if id < 0 or id > 3:
            print("id must be between [0-3]")
            return
        
        self.__write(SOLENOID, id, 0, 1 if enable else 0)

    def GPIO_write(self, id:int, value:int):
        """
        Writes data to specified GPIO pin
        
        Params
        -------
        - id : int
            - Range (0-19)
            - ID of the GPIO pin to be written to
        - value : int
            - 0 or 1
            - Value to be written to the GPIO pin
        
        """
        if (id < 0) | (id > 19):
            print("GPIO PIN SELECT MUST BE IN RANGE 0-19")
            return
        # data either a 1 or 0
        if (value == 1) | (value == 0):
            self.__write(GPIO, id, 1, value)
        else:
            print("VALUE MUST BE 0 or 1")



    def GPIO_update(self):
        """
        Triggers Payload Board to update GPIO readings

        Params
        -------
        - None

        Returns
        -------
        - None
        
        """  

        self.__write(GPIO,0, 0, 0)
        self.expect_response = True



    def DC_write(self, id:int, data:int):     
        """
        Writes data to specified DC Motor

        Params
        -------
        - id : int
            - Range (0-7)
            - ID of the DC Motor to be written to
        - data : int
            - Range (-65535-65535)
            - Speed to be written to the DC Motor
        
        """
        if ((id < 0) | (id > 7)):
            print("DC MOTOR ID MUST BE IN RANGE 0-7")
            return
        
        if ((data < -65536) | (data > 65535)):
            print("DATA MUST BE +- 65535")
            return

        if data >= 0:
            dir = 1
        else:
            dir = 0

        data = abs(data)

        self.__write(DC, id, dir, data)


    def SERVO_write(self, id:int, data:int):     
        """
        Writes data to specified SERVO Motor
        
        Params
        -------
        - id : int
            - Range [0-15]
            - ID of the SERVO Motor to be written to
        - data : int
            - Range (0-180)
            - Value to be written to SERVO
            
        """
        if (id < 0) | (id > 15):
            print("SERVO MOTOR ID MUST BE IN RANGE 0-15")
            return
        if (data < 0) | (data > 180):
            print("SERVO VALUE MUST BE IN RANGE 0-180")
            return
        # Range for Actuonix L16-140-150-6-R linear actuator is 20-180
        data = int(20+(data/180)*(180-20))
        # Mod bit is set since it works the same as the pumps
        self.__write(SERVO, id, 1, data)  



    def PUMP_write(self, id:int, data:int):
        """
        Writes data to specified PUMP Motor (Same as Servo)
        
        Params
        -------
        - id : int
            - Range 
            - ID of the PUMP (treated as SERVO) Motor to be written to
        - data : int
            - Range (0-180)
            - Value to be written to PUMP
        """ 
        if (id < 0) | (id > 15):
            print("PUMP ID MUST BE IN RANGE 0-5")   
            return     
        if (data < 0) | (data > 180):
            print("PUMP VALUE MUST BE IN RANGE 0-180")
            return
        self.__write(SERVO, id, 1, data)  



    # def STEPPER_write(self, id, mod, data):     
    #     """
    #     Writes data to specified STEPPER Motor
        
    #     """
    #     self.__write(STEPPER, id, mod, data)  


