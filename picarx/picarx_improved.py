# Filename: picarx_improved.py
# Author: Mathew Miller
# Created: January 22, 2024
# Description: This file contains an improved Python script for the Sunfounder's PiCarX code.
# Note: forward(50) ~ 20 cm/s, backward = 19 cm/s

import time
import os
import math
import logging
import atexit
import concurrent.futures
import rossros as rr
from readerwriterlock import rwlock

from logdecorator import log_on_start, log_on_end, log_on_error
logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat import reset_mcu, run_command

# Initiate data and termination busses
# bGreyIn = rr.Bus()
# bGreyOut = rr.Bus()
# bUltraIn = rr.Bus()
# bUltraOut = rr.Bus()
# bTerminate = rr.Bus(0, "Termination Bus")

# reset robot_hat
reset_mcu()
time.sleep(0.2)

def constrain(x, min_val, max_val):

    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class bus(object):

    def __init__(self):
        self.message = None  # Initialize the message attribute to None
        self.lock = rwlock.RWLockWriteD()

    def write(self, message):
        """Write a message to the bus"""
        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        """Read the message from the bus"""
        with self.lock.gen_rlock():
            return self.message

class sensing(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    def __init__(self, 
                grayscale_pins:list=['A0', 'A1', 'A2'],
                config:str=CONFIG,
                ):
        
        # --------- config_flie ---------
        self.config_flie = fileDB(config, 774, os.getlogin())

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())
    
    def stream_grayscale_data(self, grey_bus, polling_rate):
        while True:
            #logging.debug(f"Sense Setting Grey: {grey_bus.read()}")
            time.sleep(polling_rate)
            grey_bus.write(list.copy(self.grayscale.read()))

    def rr_stream(self):
        while True:
            #logging.debug(f"Sense Setting Grey: {grey_bus.read()}")
            time.sleep(polling_rate)
            grey_bus.write(list.copy(self.grayscale.read()))
        
class interp(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    #Default polarity is 1, which is dark line on light background. Use -1 for  light line on dark background 
    polarity = 1 

    def __init__(self, 
                grayscale_pins:list=['A0', 'A1', 'A2'],
                config:str=CONFIG,
                ):
        
        # --------- config_flie ---------
        self.config_flie = fileDB(config, 774, os.getlogin())

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())
    
    def get_polarity(self):
        return self.polarity
    
    def set_polarity(self, value):
        if (value == 1) or (value == -1):
            self.polarity = value
        else:
            logging.error("Incorrect Polarity Value")

    def get_calc_contrast(self, greys):
        #Default output is 0 or "centered"
        output = 0
        
        #Threshold 
        thresh = 0.2

        #greys = self.get_grayscale_data()
        #if isinstance(greys, list) and len(greys) == 3:
        if greys != None:
            #greys = self.get_grayscale_data()
            grey_avg = (greys[0] + greys[1] + greys[2])/3
            greys_norm = [(greys[0]/grey_avg), (greys[1]/grey_avg), (greys[2]/grey_avg)]
            greys_thresh = [greys_norm[1] - greys_norm[0], greys_norm[1] - greys_norm[2]]

            #Sets the steering value/output 
            #Case 1: We are far left/Make a sharp right turn (left diff = 0, right diff = positive)
            if (0 < abs(greys_thresh[0]) < thresh) and (abs(greys_thresh[1]) > thresh) and (greys_thresh[1] > 0): 
                #Output 1 = turn sharp right (001)
                output = 1
            #Case 2: We are slight left/need to turn slight right (left diff = negative, right diff = 0)
            elif (abs(greys_thresh[0]) > thresh) and (0 < abs(greys_thresh[1]) < thresh) and (greys_thresh[0] < 0):
                #Output 0.5 = turn slight right (011)
                output = 0.5
            #Case 3: We are slight right/need to turn slight left (left diff = 0, right diff = negative)
            elif (0 < abs(greys_thresh[0]) < thresh) and (abs(greys_thresh[1]) > thresh) and (greys_thresh[1] < 0): 
                #Output -0.5 = turn slight left (110)
                output = -0.5
            #Case 4: We are far right/need to turn sharp left (left diff = positive, right diff = 0)
            elif (abs(greys_thresh[0]) > thresh) and (0 < abs(greys_thresh[1]) < thresh) and (greys_thresh[0] > 0):
                #Output -1 = turn sharp left (100)
                output = -1 
            #Case 5: Were centered, go straight
            else:
                #Output 0 = centered (010)
                output = 0
        else:
            raise ValueError("grayscale reference must be a 1*3 list")
        
        #For Debugging 
        logging.debug(f"Greys: {greys}")
        # logging.debug(f"Diffs: {greys_thresh}")
        # logging.debug(f"Normalized Diffs: {greys_norm_thresh}")
        logging.debug(f"Output State: {output}")

        return (output * self.polarity)
    
    def transform_greyscale(self, grey_bus, move_bus, polling_rate):
        #Default output is 0 or "centered"
        output = 0
        
        #Threshold 
        thresh = 0.2

        while True: 
            logging.debug(f"Inter Input Grey: {grey_bus.read()}")
            logging.debug(f"Inter Output Move: {output}")
            greys = grey_bus.read()
            if greys != None:
                
                grey_avg = (greys[0] + greys[1] + greys[2])/3
                greys_norm = [(greys[0]/grey_avg), (greys[1]/grey_avg), (greys[2]/grey_avg)]
                greys_thresh = [greys_norm[1] - greys_norm[0], greys_norm[1] - greys_norm[2]]

                #Sets the steering value/output 
                #Case 1: We are far left/Make a sharp right turn (left diff = 0, right diff = positive)
                if (0 < abs(greys_thresh[0]) < thresh) and (abs(greys_thresh[1]) > thresh) and (greys_thresh[1] > 0): 
                    #Output 1 = turn sharp right (001)
                    output = 1
                #Case 2: We are slight left/need to turn slight right (left diff = negative, right diff = 0)
                elif (abs(greys_thresh[0]) > thresh) and (0 < abs(greys_thresh[1]) < thresh) and (greys_thresh[0] < 0):
                    #Output 0.5 = turn slight right (011)
                    output = 0.5
                #Case 3: We are slight right/need to turn slight left (left diff = 0, right diff = negative)
                elif (0 < abs(greys_thresh[0]) < thresh) and (abs(greys_thresh[1]) > thresh) and (greys_thresh[1] < 0): 
                    #Output -0.5 = turn slight left (110)
                    output = -0.5
                #Case 4: We are far right/need to turn sharp left (left diff = positive, right diff = 0)
                elif (abs(greys_thresh[0]) > thresh) and (0 < abs(greys_thresh[1]) < thresh) and (greys_thresh[0] > 0):
                    #Output -1 = turn sharp left (100)
                    output = -1 
                #Case 5: Were centered, go straight
                else:
                    #Output 0 = centered (010)
                    output = 0

            time.sleep(polling_rate)

            if greys != None:
                move_bus.write(output * self.polarity)
    
class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P4'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 774, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))
        
        atexit.register(self.stop)

    #This is the function used for the first manuver on the PiCar. It takes in a (bool, int, int, int, int). The bool is used to choose forward or backwards
    def line_movement(self, forward, angle, speed, timer):
        #This is used if the user wants the car to start moving forward
        if forward:
            self.set_dir_servo_angle(angle)
            self.forward(speed)
            time.sleep(timer)
            self.stop()
            time.sleep(0.5)
            self.backward(speed)
            time.sleep(timer)
            self.stop()
            self.set_dir_servo_angle(0)
        #This is used if the user wants the car to start moving backwards
        else:
            self.set_dir_servo_angle(angle)
            self.backward(speed)
            time.sleep(timer)
            self.stop()
            time.sleep(0.5)
            self.forward(speed)
            time.sleep(timer)
            self.stop()
            self.set_dir_servo_angle(0)

    #This is the function for parallel parking the car. The input 'right' is a bool that chooses wither the car parks to the right (True) or left (False)
    def parallel(self, right):
        speed = 50
        angle = 25
        straightTimer = 0.15
        turnTimer = 1.2
        parkbackTimer = 0
        parkforTimer = 0.75
        #Start with the wheels straight
        self.set_dir_servo_angle(0)
        time.sleep(0.5)

        #Parking to the right
        if right:
            self.backward(speed)
            time.sleep(straightTimer)
            self.set_dir_servo_angle(angle)
            time.sleep(turnTimer)
            self.set_dir_servo_angle(-angle)
            time.sleep(turnTimer + 0.1)
            self.set_dir_servo_angle(0)
            self.backward(speed)
            time.sleep(parkbackTimer)
            self.forward(speed)
            time.sleep(parkforTimer)

        #Parking to the left
        else:
            self.backward(speed)
            time.sleep(straightTimer)
            self.set_dir_servo_angle(-angle)
            time.sleep(turnTimer)
            self.set_dir_servo_angle(angle)
            time.sleep(turnTimer)
            self.set_dir_servo_angle(0)
            self.backward(speed)
            time.sleep(parkbackTimer)
            self.forward(speed)
            time.sleep(parkforTimer)

        #Stop the car after its in position
        self.stop()

    #This is the function for a three point turn. The input 'right' is a bool that chooses wither the car parks to the right (True) or left (False)
    def kturn(self, right):
        speed = 50
        angle = 19.5
        timer = 2.4
        backOffset = 0
        forwardOffset = 0
        horizontalOffset = 0.3
        #Start with the wheels straight
        self.set_dir_servo_angle(0)
        time.sleep(1)

        #Turning to the right 
        if right:
            self.forward(speed)
            self.set_dir_servo_angle(angle)
            time.sleep(timer)
            self.set_dir_servo_angle(0)
            self.backward(speed)
            time.sleep(horizontalOffset)
            self.set_dir_servo_angle(-angle)
            self.backward(speed)
            time.sleep(timer + backOffset)
            self.set_dir_servo_angle(0)
            self.forward(speed)
            time.sleep(timer + forwardOffset)

        #Turning to the left
        else:
            self.forward(speed)
            self.set_dir_servo_angle(-angle)
            time.sleep(timer)
            self.backward(speed)
            self.set_dir_servo_angle(angle)
            time.sleep(timer + backOffset)
            self.set_dir_servo_angle(0)
            self.forward(speed)
            time.sleep(timer + forwardOffset)
        

    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        #if speed != 0:
            #speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = math.cos(math.radians(current_angle))
            logging.debug("Power Scale: %s", power_scale)
            #power_scale = (100 - abs_current_angle) / 100.0 
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = math.cos(math.radians(current_angle))
            logging.debug("Power Scale: %s", power_scale)
            #power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")
        
    def handle_input(self, user_input):

        #Forward/Backward Input Tree
        if user_input.lower() == '1' or user_input.lower() == 'line movement':
            #Set Forward or Backwards
            user_input = input("Would you like to go forward or backward (f/b): ")
            if user_input.lower() == 'f' or user_input.lower() == 'forward':
                forward = True
            elif user_input.lower() == 'b' or user_input.lower() == 'backward':
                forward = False
            else:
                print("Invalid input. Please enter 'f' or 'b' next time. The default of 'forward' will be chosen for this time.")
                forward = True

            #Angle input
            user_input = input("Enter an angle in degrees from -30 to 30 if you would like. Enter 0 if you want a straight line: ")
            try:
                angle = int(user_input)
                if -30 <= angle <= 30:
                    #A good angle was entered, nothing to change
                    angle = angle
                else:
                    print("Invalid angle. Please enter an angle between -30 and 30 next time. The default of '0' will be chosen for this time.")
                    angle = 0
            except ValueError:
                print("Invalid angle. Please enter an angle between -30 and 30 next time. The default of '0' will be chosen for this time.")
                angle = 0

            #Speed input
            user_input = input("Enter a speed from 0 to 100 (100 is full speed): ")
            speed = int(user_input)
            if 0 <= angle <= 100:
                #A good speed was entered, nothing to change
                speed = speed
            else:
                print("Invalid speed. Please enter an speed between 0 and 100 next time. The default of '50' will be chosen for this time.")
                speed = 50

            #Timer input
            user_input = input("Enter a time in seconds you would like to run for (max time is 60 seconds): ")
            timer = int(user_input)
            if 0 <= angle <= 60:
                #A good speed was entered, nothing to change
                timer = timer
            else:
                print("Invalid time. Please enter an time between 0 and 60 seconds next time. The default of '5' seconds will be chosen for this time.")
                timer = 5

            self.line_movement(forward, angle, speed, timer)
        
        #Parallel Parking Input
        elif user_input.lower() == '2' or user_input.lower() == 'parallel parking':
            user_input = input("Would you like to park to the left or the right? (r/l): ")
            if user_input.lower() == 'r' or user_input.lower() == 'right':
                right = True
            elif user_input.lower() == 'l' or user_input.lower() == 'left':
                forward = False
            else:
                print("Invalid input. Please enter 'r' or 'l' next time. The default of 'right' will be chosen for this time.")
                right = True
            self.parallel(right)

        #Three Point Turn Input
        elif user_input.lower() == '3' or user_input.lower() == 'three point turn':
            user_input = input("Would you like to turn to the left or the right initially? (r/l): ")
            if user_input.lower() == 'r' or user_input.lower() == 'right':
                right = True
            elif user_input.lower() == 'l' or user_input.lower() == 'left':
                forward = False
            else:
                print("Invalid input. Please enter 'r' or 'l' next time. The default of 'right' will be chosen for this time.")
                right = True
            self.kturn(right)

        #Handle the 'exit' condition
        elif user_input.lower() == 'exit':
            print("Exiting the program.")
            exit()

        #Invalid input
        else:
            print("Invalid input. Please enter '1', '2', '3', or 'exit'.")

    def dodge_this(self):
    
        dist = self.get_distance()
        logging.debug(f"Distance Read: {dist}")
        if 0 < dist < 20:
            obj = True
        else:
            obj = False
            
        return obj

class control(Picarx):
    #Set cont_in to something we'll never get
    cont_in = 100

    #Angles
    slight = 13
    hard = 30

    def init(self):
        super().init()
        atexit.register(self.stop)

    def set_cont_in(self, value):
        if (-1 <= value <= 1):
            self.cont_in = value
        else:
            logging.error("Incorrect Input Recived by Controller")

    def drive(self, value, obj):
        self.cont_in = value
        if self.cont_in != 100 and not obj:
            if self.cont_in == -1:
                self.set_dir_servo_angle(-self.hard)
            elif self.cont_in == -0.5:
                self.set_dir_servo_angle(-self.slight)
            elif self.cont_in == 0:
                self.set_dir_servo_angle(0)
            elif self.cont_in == 0.5:
                self.set_dir_servo_angle(self.slight)
            elif self.cont_in == 1:
                self.set_dir_servo_angle(self.hard)
            else:
                logging.error("Something went wrong when setting the stearing angle")

            self.forward(35)
        elif obj:
            self.stop()

    def consume_drive(self, move_bus, polling_rate):

        while True:
            self.cont_in = move_bus.read()
            logging.debug(f"Move: {move_bus.read()}")

            if self.cont_in != 100 and self.cont_in != None:
                if self.cont_in == -1:
                    self.set_dir_servo_angle(-self.hard)
                elif self.cont_in == -0.5:
                    self.set_dir_servo_angle(-self.slight)
                elif self.cont_in == 0:
                    self.set_dir_servo_angle(0)
                elif self.cont_in == 0.5:
                    self.set_dir_servo_angle(self.slight)
                elif self.cont_in == 1:
                    self.set_dir_servo_angle(self.hard)
                else:
                    logging.error("Something went wrong when setting the stearing angle")

                self.forward(35)
            time.sleep(polling_rate)


if __name__ == "__main__":
    px = Picarx()
    sense = sensing()
    inter = interp()
    controller = control()

    # move_bus = bus()
    # grey_bus = bus()

    bSensor = rr.Bus(sense.get_grayscale_data(), "Grey Scale Bus")
    bInterpret = rr.Bus(inter.get_calc_contrast(sense.get_grayscale_data()), "Interpret Bus")
    bUltra = rr.Bus(px.dodge_this(), "Obstacle Bus")
    bControl = rr.Bus(controller.drive(inter.get_calc_contrast(sense.get_grayscale_data()), px.dodge_this()), "Control bus")
    bTerminate = rr.Bus(0, "Termination Bus")

    # Wrap the square wave signal generator into a producer
    readGrey = rr.Producer(
        sense.get_grayscale_data,  # function that will generate data
        bSensor,  # output data bus
        0.05,  # delay between data generation cycles
        bTerminate,  # bus to watch for termination signal
        "Read the greyscale data")

    # Wrap the square wave signal generator into a producer
    readUltra = rr.Producer(
        px.dodge_this,  # function that will generate data
        bUltra,  # output data bus
        0.05,  # delay between data generation cycles
        bTerminate,  # bus to watch for termination signal
        "Read the ultrasonic data")


    # Wrap the multiplier function into a consumer-producer
    interpCalc = rr.ConsumerProducer(
        inter.get_calc_contrast,  # function that will process data
        bSensor,  # input data buses
        bInterpret,  # output data bus
        0.05,  # delay between data control cycles
        bTerminate,  # bus to watch for termination signal
        "Calculate the greyscale turning angle")
    
    # Wrap the data controlCar into a consumer
    controlCar = rr.Consumer(
            controller.drive,  # function that will process data
            (bInterpret, bUltra),  # input data buses
            0.05,  # delay between data control cycles
            bTerminate,  # bus to watch for termination signal
            "Control PiCar")
    """ Fourth Part: Create RossROS Printer and Timer objects """

    # Make a printer that returns the most recent wave and product values
    printBuses = rr.Printer(
        (bSensor, bUltra, bInterpret, bControl, bTerminate),  # input data buses
        # bMultiplied,      # input data buses
        0.25,  # delay between printing cycles
        bTerminate,  # bus to watch for termination signal
        "Print raw and derived data",  # Name of printer
        "Data bus readings are: ")  # Prefix for output
    
    # Make a timer (a special kind of producer) that turns on the termination
    # bus when it triggers
    terminationTimer = rr.Timer(
            bTerminate,  # Output data bus
            10,  # Duration
            0.01,  # Delay between checking for termination time
            bTerminate,  # Bus to check for termination signal
            "Termination timer")  # Name of this timer

    # Create a list of producer-consumers to execute concurrently
    producer_consumer_list = [readGrey,
                                interpCalc,
                                controlCar,
                                readUltra,
                                printBuses, 
                                terminationTimer]
    
    #Execute the list of producer-consumers concurrently
    rr.runConcurrently(producer_consumer_list)

    atexit.register(px.stop)

    # logging.debug(f"Grey: {grey_bus.read()}")
    # logging.debug(f"Move: {move_bus.read()}")

    # with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
    #     eSensor = executor.submit(sense.stream_grayscale_data, grey_bus,
    #     0.01)
    #     eInterpreter = executor.submit(inter.transform_greyscale,
    #     grey_bus, move_bus, 0.02)
    #     eController = executor.submit(controller.consume_drive,
    #     move_bus, 0.03)

    #     eSensor.daemon = True
    #     eInterpreter.daemon = True
    #     eController.daemon = True 


    # while True:

    #     grey = sense.get_grayscale_data()
    #     interps = inter.get_calc_contrast(grey)
    #     cont = controller.drive(interps, px.dodge_this())

        #Week 3 Line Following

        #Incoming data from the greyscale sensor
        # logging.debug(f"Grey: {grey_bus.read()}")
        # logging.debug(f"Move: {move_bus.read()}")
        #Week 4 Consumer Producer 
        
        # sense.stream_grayscale_data(grey_bus, 0.1)  
        # inter.transform_greyscale(grey_bus, move_bus, 0.1)
        # controller.consume_drive(move_bus, 0.1)

        # #Sets the greyscale in interp so that it can be refrenced later (might be un-needed)
        # inter.set_grayscale_reference(sense.get_grayscale_data())

        # #This is where the direction is calculated for turns and movement 
        # controller.set_cont_in(inter.get_calc_contrast())

        # #This sets the polarity, 1 means black line on light background, -1 means white line on dark background
        # inter.set_polarity(1)

        # #This takes the calculation from 
        # controller.drive()

        #Used to slow down the loop if desired
        #time.sleep(0.1)

        #Week 2 Manuver 
        # user_input = input("Enter maneuver ('1'/'Line Movement', '2'/'Parallel Parking', '3'/'Three Point Turn') or 'exit' to quit: ")
        # px.handle_input(user_input)
        # px.stop()

    # px.stop()
