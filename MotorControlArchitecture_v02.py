#!/usr/bin/env python3
import time
import numpy as np
from pololu_drv8835_rpi import motors, MAX_SPEED
import threading

class quadrature_encoder:
    def __init__(self, serial_string_identifier, counts_per_encoder_revolution, encoder_to_output_shaft_multiplier, shaft_radians_to_output_units_multiplier, forward_encoder_rotation_sign) :  # serial_string_identifier is e.g. "E0" or "E1"
        self.serial_string_identifier = serial_string_identifier  # does nothing, just an identifier that we hope the user will keep as a unique link to the sensor data streaming in. 
        self.counts_per_encoder_revolution = counts_per_encoder_revolution
        self.encoder_to_output_shaft_multiplier = encoder_to_output_shaft_multiplier
        self.shaft_radians_to_output_units_multiplier = shaft_radians_to_output_units_multiplier  # e.g. a wheel radius to get linear displacement and speed
        self.forward_encoder_rotation_sign = forward_encoder_rotation_sign
        
        self.counts_to_radians_multiplier = float(1)/self.counts_per_encoder_revolution * self.encoder_to_output_shaft_multiplier * 2*np.pi * self.forward_encoder_rotation_sign 
        
        self.t = time.perf_counter()
        self.t_previous = self.t
        self.dt = self.t - self.t_previous
        self.counts = 0
        self.radians = 0
        self.output_units = 0 
        self.counts_velocity = 0
        self.radians_velocity = 0
        self.output_units_velocity = 0 
        
        self.data_are_new = 0   # this flag will be returned along with any data to signify whether the data are updated or not. 
        
        self.is_new = True  # this is a flag that will make "update" run twice (to set both position and velocity) the first time it is called. 

        
    def update(self, counts_measured):
        self.t_previous = self.t
        self.t = time.perf_counter()
        self.dt = self.t - self.t_previous
        
        if self.is_new:
            self.counts_offset = counts_measured
            self.is_new = False
            self.update(self.counts) # call with the number of counts already present
        
        self.counts_previous = self.counts
        self.counts = counts_measured - self.counts_offset
        self.radians_previous = self.radians
        self.radians = self.counts * self.counts_to_radians_multiplier
        self.output_units_previous = self.output_units 
        self.output_units = self.radians * self.shaft_radians_to_output_units_multiplier 
        
        self.counts_velocity_previous = self.counts_velocity
        self.counts_velocity = float(self.counts - self.counts_previous)/self.dt
        self.radians_velocity_previous = self.radians_velocity
        self.radians_velocity = self.counts_velocity * self.counts_to_radians_multiplier
        self.output_units_velocity_previous = self.output_units_velocity
        self.output_units_velocity = self.radians_velocity * self.shaft_radians_to_output_units_multiplier 
                
        self.data_are_new = True
        
    def get_counts(self):
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.counts, self.dt, report_data_as_new
        
    def get_counts_velocity(self): 
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.counts_velocity, self.dt, report_data_as_new
        
    def get_radians(self): 
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.radians, self.dt, report_data_as_new
        
    def get_radians_velocity(self): 
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.radians_velocity, self.dt, report_data_as_new
        
    def get_output_units(self): 
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.output_units, self.dt, report_data_as_new
        
    def get_output_units_velocity(self): 
        report_data_as_new = self.data_are_new
        self.data_are_new = False   # If this is called before an update, it needs to report that it is not new.
        return self.output_units_velocity, self.dt, report_data_as_new
        

# a general purpose PID controller, to be instantiated in whatever units the user desires. 
# Implemented as a Thread (threading.Thread) to make it run continuously on its own. 
# To run it, just create the object and call its .start() method. 
class PID_controller (threading.Thread):   
    def __init__(self,motor,encoder_getdata_function,Kp,Ki,Kd,error_integral_limit=np.inf,forward_motor_command_sign=1): 
        threading.Thread.__init__(self)
        self.motor = motor
        self.encoder_getdata_function = encoder_getdata_function
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.error_integral_limit = error_integral_limit
        self.forward_motor_command_sign = forward_motor_command_sign
        
        # dummy values
        self.dt = 0.01     
        self.target = 0
        self.error = 0
        self.error_previous = 0
        self.error_integral = 0
        self.error_derivative = 0
        
        self.motor_command = 0
        
    def compute_error(self):
        self.error_previous = self.error 
        self.error = self.target - self.current
        
    def compute_error_integral(self):
        self.error_integral += self.error * self.dt
        # apply the error_integral_limit
        if self.error_integral > abs(self.error_integral_limit):
            self.error_integral = abs(self.error_integral_limit)
        elif self.error_integral < -abs(self.error_integral_limit):
            self.error_integral = -abs(self.error_integral_limit)
            
    def compute_error_derivative(self): 
        self.error_derivative = float(self.error - self.error_previous)/self.dt
        
    def compute_command(self): 
        self.motor_command = self.forward_motor_command_sign * (self.Kp*self.error + self.Ki*self.error_integral + self.Kd*self.error_derivative)
        
    def command_motor(self): 
        self.motor.setSpeed(int(self.motor_command))
        
    def update(self):
        self.current, self.dt, data_are_new = self.encoder_getdata_function() # current value
        
        if data_are_new:
            self.compute_error()
            self.compute_error_integral()
            self.compute_error_derivative()
            self.compute_command()
            self.command_motor()
        
    def set_target(self,target): 
        self.target = target # target value
        self.error_integral = 0
        self.update()     
#         print([self.current])
        
    def run(self):
        self.error_integral = 0
        while True: 
            self.update()
            time.sleep(0.001)

            
            
        
##%% Example code to test quadrature counting. 
#counts_per_encoder_revolution = 12
#encoder_to_output_shaft_multiplier = (float(13)/34 * float(12)/34 * float(13)/35 * float(10)/38)  # roughly 75.81
#shaft_radians_to_output_units_ratio = 0.030 # 1 radian makes this many linear meters
##Encoder object instantiation:  quadrature_encoder(serial_string_identifier, counts_per_encoder_revolution, output_gear_reduction_ratio, shaft_radians_to_output_units_ratio, forward_encoder_rotation_sign)
#encoder_left = quadrature_encoder("E0",counts_per_encoder_revolution, encoder_to_output_shaft_multiplier, shaft_radians_to_output_units_ratio, 1)
#controller_left_position_radians = PID_controller(motors.motor1, encoder_left.get_radians, 200,0,0,np.inf,1)
#controller_left_position_meters = PID_controller(motors.motor1, encoder_left.get_output_units, 6666.66,0,0,np.inf,1)
#controller_left_velocity_rad_per_second = PID_controller(motors.motor1, encoder_left.get_radians_velocity, 20,0,0,np.inf,1)
#controller_left_velocity_meters_per_second = PID_controller(motors.motor1, encoder_left.get_output_units, 60, 0, 0, np.inf, 1)
#
#for ii in range(0,10,1):
#    encoder_left.update(ii)
#    print("radians {0}, radvel {1}, dt {2}".format(encoder_left.radians, encoder_left.radians_velocity, encoder_left.dt) )
#    time.sleep(0.1)
#    
#controller_left_position_radians.set_target(2*np.pi)
#time.sleep(2)
#    
#motors.motor1.setSpeed(0)

