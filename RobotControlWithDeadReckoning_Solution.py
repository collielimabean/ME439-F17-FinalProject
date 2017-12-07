#!/usr/bin/env python3

import MotorControlArchitecture_v03 as mc
import ME439_Sensors
import ME439_Robot
from pololu_drv8835_rpi import motors
import numpy as np
import time
import pickle


try:
    # First turn off the motors
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    
    # now get a "ME439_Sensors.sensor_suite" object. 
    robot_sensors = ME439_Sensors.sensor_suite(saveFile=0)
    robot_sensors.start()
    
    # now set up a Robot object
    wheel_width = 0.127
    body_length = 0.155
    robot0 = ME439_Robot.robot(wheel_width,body_length)
    
    # Choose which encoder is Left and Right    
    encoder_left = robot_sensors.encoder_0
    encoder_right = robot_sensors.encoder_1
    
    # Set up a variety of controllers you could use. Note the PID settings!
    # Several template controllers - Left wheel
    controller_left_position_radians = mc.PID_controller(motors.motor1, encoder_left.get_radians, 200,0,0,error_integral_limit=np.inf,forward_motor_command_sign=1)
    controller_left_position_meters = mc.PID_controller(motors.motor1, encoder_left.get_output_units, 6000,0,0,error_integral_limit=np.inf,forward_motor_command_sign=1)
    controller_left_velocity_rad_per_second = mc.PID_controller(motors.motor1, encoder_left.get_radians_velocity, 30,20,0,error_integral_limit=6,forward_motor_command_sign=1)
    controller_left_velocity_meters_per_second = mc.PID_controller(motors.motor1, encoder_left.get_output_units_velocity, 900, 15000, 0, error_integral_limit=0.18,forward_motor_command_sign=1)
    
    # Several template controllers - Right wheel
    controller_right_position_radians = mc.PID_controller(motors.motor2, encoder_right.get_radians, 200,0,0,error_integral_limit=np.inf,forward_motor_command_sign=-1)
    controller_right_position_meters = mc.PID_controller(motors.motor2, encoder_right.get_output_units, 6000,0,0,error_integral_limit=np.inf,forward_motor_command_sign=-1)
    controller_right_velocity_rad_per_second = mc.PID_controller(motors.motor2, encoder_right.get_radians_velocity, 30,20,0,error_integral_limit=6,forward_motor_command_sign=-1)
    controller_right_velocity_meters_per_second = mc.PID_controller(motors.motor2, encoder_right.get_output_units_velocity, 900, 15000, 0, error_integral_limit=0.18,forward_motor_command_sign=-1)
    
    # Select controller
    active_controller_left = controller_left_velocity_meters_per_second
    active_controller_right = controller_right_velocity_meters_per_second

    # Start the Controllers with reasonable targets (like 0)
    active_controller_left.set_target(0)
    active_controller_right.set_target(0)
    time.sleep(0.05)    # This short delay gives time for the encoder signals to update
    active_controller_left.start()
    active_controller_right.start()
    
    ### RECOMMENDED INITIAL SPEED SETTINGS (If using a speed controller)
    ### 2*np.pi  Radians per second
    ### roughly 0.19 meters per second
    print("Controllers On")

    # NEXT program a series of maneuvers, e.g. Duration, Left_Speed, Right_Speed
    # And in the While loop, update to the next stage at the appropriate time.
    
    # Down and Back
    stage_settings = np.array([[1,0,0],robot0.plan_line(0.2, 0.5),robot0.plan_pivot(-1,-np.pi),robot0.plan_line(0.2, 0.5),robot0.plan_pivot(1,np.pi)])
    
    # Letter "P"
    stage_settings = np.array([[0.1,0,0],robot0.plan_pivot(-1,-np.pi/12), robot0.plan_line(0.3,1), robot0.plan_pivot(-1,-np.pi*5/12), robot0.plan_arc(-0.3,0.3,-np.pi), robot0.plan_line(0.3,0.15), robot0.plan_pivot(1,np.pi*5/12), robot0.plan_line(0.3,0.378834), robot0.plan_pivot(1,-np.pi*11/12)])

    stage = 0   # initialize in the 0th stage  
    active_controller_left.set_target(stage_settings[stage,1])
    active_controller_right.set_target(stage_settings[stage,2])
    
    print('Stage 0 set')

    t_stage_start = time.clock() # time.perf_counter() will give a reliable elapsed time. 
    
    # loop to continually execute the controller: 
    while True:
        # Maybe do things like checking the time or the displacement and updating the settings. 
#        active_controller.set_target(2*np.pi)

        # Apparently this Delay is critical. I don't know why yet. 
        time.sleep(0.005)
        t_current = time.clock()
#        print(t_current - t_stage_start)
        if (t_current-t_stage_start) >= stage_settings[stage,0]: 
            print('Time Detected')
            print(stage_settings[stage,:])
            stage += 1   
            if stage >= stage_settings.shape[0]:    # If the stage is set to a higher number than exists...then shut down and exit
                stage = stage_settings.shape[0] - 1
                active_controller_left.set_target(0)
                active_controller_right.set_target(0)
                break
            active_controller_left.set_target(stage_settings[stage,1])
            active_controller_right.set_target(stage_settings[stage,2])
            t_stage_start = t_current # reset the stage start time to present

        robot0.update_encoders(encoder_left.output_units, encoder_right.output_units)
        robot0.dead_reckoning()
        
    # After finished, shut down the motors.         
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)            
    
    # SAVE the Dead Reckoning data in a Pickle file. 
    filename = "DeadReckData_{0}.p".format(time.strftime("%Y%m%d-%H_%M_%S"))
    pickle.dump(robot0, open(filename,"wb") )
    print("Dead Reckoning Data saved as:    {0}".format(filename))

except KeyboardInterrupt: 
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    
    