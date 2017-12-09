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

# constants #
#WORKSPACE_WIDTH = 1.22
#WORKSPACE_LENGTH = 2.12
#NODE_GRID_WIDTH = 100
#NODE_GRID_LENGTH = 100

class Node:
    def __init__(self, node_x, node_y, number_nodes_x, number_nodes_y, actual_width, actual_length):
        self.node_x = node_x
        self.node_y = node_y
        self.actual_x = (actual_width / number_nodes_x) * node_x
        self.actual_y = (actual_length / number_nodes_y) * node_y
        self.vector = np.array([0, 0])

class Obstacle:
    def __init__(self, node_x, node_y, number_nodes_x, number_nodes_y, actual_width, actual_length, radius):
        self.node_x = node_x
        self.node_y = node_y
        self.actual_x = (actual_width / number_nodes_x) * node_x
        self.actual_y = (actual_length / number_nodes_y) * node_y
        self.radius = radius # m

# configuration 
workspace_width = 1.22 # m
workspace_length = 2.12 # m

node_grid_width = 25
node_grid_length = 50

start_x = 0
start_y = 0

end_node_x = node_grid_width - 1
end_node_y = node_grid_length - 1

endpoint_scaling_factor = 5.0
repulsion_scaling_factor = 25.0

# grid & obstacle initialization
node_grid = [[Node(i, j, node_grid_width, node_grid_length, workspace_width, workspace_length) for j in range(node_grid_length)] for i in range(node_grid_width)]

obstacles = [
    Obstacle(15, 15, node_grid_width, node_grid_length, workspace_width, workspace_length, 0.01),
    Obstacle(5, 25, node_grid_width, node_grid_length, workspace_width, workspace_length, 0.01),
    Obstacle(17, 35, node_grid_width, node_grid_length, workspace_width, workspace_length, 0.01)
]

# compute vector field
for i in range(len(node_grid)):
    for j in range(len(node_grid[i])):
        n = node_grid[i][j]

        # initialization
        resultant_vector = np.array([0.0, 0.0])

        # check if node is on or inside a node
        node_is_inside = False
        for obstacle in obstacles:
            if ((n.actual_x - obstacle.actual_x) ** 2 + (n.actual_y - obstacle.actual_y) ** 2) < obstacle.radius:
                node_is_inside = True
                break
        
        if node_is_inside:
            continue

        # normalize to unit vector & scale
        node_to_end = np.array([end_node_x - i, end_node_y - j])

        # verify that the vector is not (0, 0)
        if node_to_end.any():
            scaled_node_to_end_vector = (endpoint_scaling_factor / np.linalg.norm(node_to_end)) * node_to_end

            #print '[%d, %d]: %s' % (i, j, str(scaled_node_to_end_vector))

            # add node to endpoint vector to resultant
            resultant_vector += scaled_node_to_end_vector

        # for each obstacle, compute repulsion vector and add to resultant
        for obstacle in obstacles:
            obstacle_vector = np.array([i - obstacle.node_x, j - obstacle.node_y])

            # verify vector is not (0, 0)
            if obstacle_vector.any():
                scaled_obstacle_vector = (repulsion_scaling_factor / (np.linalg.norm(obstacle_vector) ** 3)) * obstacle_vector
                resultant_vector += scaled_obstacle_vector

        # set the node object's vector
        n.vector = resultant_vector

# plot the vector field #
# Y is at location 0, X is at 1
X, Y = np.meshgrid(np.arange(0, node_grid_length), np.arange(0, node_grid_width))
U = [[node.vector[1] for node in row] for row in node_grid]
V = [[node.vector[0] for node in row] for row in node_grid]

plt.figure()
plt.title('some jank stuff')
Q = plt.quiver(X, Y, U, V, units='width')
plt.show()


# determining correct commands to feed the robot as it travels #



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