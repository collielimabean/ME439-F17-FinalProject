#!/usr/bin/env python3

"""
import MotorControlArchitecture_v03 as mc
import ME439_Sensors
import ME439_Robot
from pololu_drv8835_rpi import motors
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import pickle

class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

# location size
WORKSPACE_WIDTH = 1.22 # m
WORKSPACE_LENGTH = 2.12 # m

# grid resolution
NODE_GRID_WIDTH = 50
NODE_GRID_LENGTH = 50

# start position, (0, 0) is lower left corner
START_X = 0 # m
START_Y = 0 # m

# end position (0, 0) is upper right corner
END_X = 1.22 # m
END_Y = 2.12 # m

# scaling factors
ENDPOINT_SCALING_FACTOR = 1
REPULSION_SCALING_FACTOR = 0.25

# obstacle locations from lower left corner
OBSTACLES = [
    Obstacle(1, 1, 0.1)
]

def main():
    x_linspace = np.linspace(0, WORKSPACE_WIDTH, NODE_GRID_WIDTH)
    y_linspace = np.linspace(0, WORKSPACE_LENGTH, NODE_GRID_LENGTH)
    x_nodes, y_nodes = np.meshgrid(x_linspace, y_linspace)

    # this is a 3D matrix. Instead of having matrix[i, j] being a 
    # scalar value like 3, matrix[i, j] holds an array of size 2.
    # in other words, each element in the matrix holds a vector
    node_vectors = np.zeros((NODE_GRID_LENGTH, NODE_GRID_WIDTH, 2))
    for (i, j) in np.ndindex(NODE_GRID_LENGTH, NODE_GRID_WIDTH):
        x_loc = x_linspace[j]
        y_loc = y_linspace[i]

        resultant_vector = np.zeros(2)

        # check if node is on or inside a node
        # if so, vector field is 0
        node_is_inside = False
        for obstacle in OBSTACLES:
            if ((x_loc - obstacle.x) ** 2 + (y_loc - obstacle.y) ** 2) < obstacle.radius:
                node_is_inside = True
                break

        if node_is_inside:
            continue

        # normalize to unit vector & scale
        node_to_end = np.array([END_X - x_loc, END_Y - y_loc])

        # verify that the vector is not (0, 0)
        if node_to_end.any():
            scale_factor = ENDPOINT_SCALING_FACTOR / np.linalg.norm(node_to_end)
            scaled_node_to_end_vector = scale_factor * node_to_end

            # add node to endpoint vector to resultant
            resultant_vector += scaled_node_to_end_vector

        # for each obstacle, compute repulsion vector and add to resultant
        for obstacle in OBSTACLES:
            obstacle_vector = np.array([x_loc - obstacle.x, y_loc - obstacle.y])

            # verify vector is not (0, 0)
            if obstacle_vector.any():
                scale_factor = REPULSION_SCALING_FACTOR / (np.linalg.norm(obstacle_vector) ** 3)
                scaled_obstacle_vector = scale_factor * obstacle_vector
                resultant_vector += scaled_obstacle_vector

        # set resultant vector
        #print(i, j, node_to_end, resultant_vector)
        node_vectors[i, j] = resultant_vector

    # pull x/y components out for plotting
    vector_field_x_comp = node_vectors[:, :, 0]
    vector_field_y_comp = node_vectors[:, :, 1]

    plt.figure()
    plt.title('Vector Field')
    plt.quiver(x_nodes, y_nodes, vector_field_x_comp, vector_field_y_comp, units='width')
    plt.show()


    # determining correct commands to feed the robot as it travels #



if __name__ == '__main__':
    main()


"""
try:
    # First turn off the motors
    motors.motor1.setSpeed(0)
    motors.motor2.setSpeed(0)
    
    # now get a "ME439_Sensors.sensor_suite" object. 
    robot_sensors = ME439_Sensors.sensor_suite(saveFile=0)
    robot_sensors.start()
    
    # now set up a Robot object
    wheel_diameter = 0.0596 # m
    wheel_width = 0.133 # m
    body_length = 0.155 # m
    robot0 = ME439_Robot.robot(wheel_width, body_length)
    
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

    stage = 0   # initialize in the 0th stage  
    active_controller_left.set_target(stage_settings[stage, 1])
    active_controller_right.set_target(stage_settings[stage, 2])

    t_stage_start = time.clock() # time.perf_counter() will give a reliable elapsed time. 
    
    # loop to continually execute the controller: 
    while True:
        # Maybe do things like checking the time or the displacement and updating the settings. 
#        active_controller.set_target(2*np.pi)

        # Apparently this Delay is critical. I don't know why yet. 
        time.sleep(0.005)
        t_current = time.clock()
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
    
"""    