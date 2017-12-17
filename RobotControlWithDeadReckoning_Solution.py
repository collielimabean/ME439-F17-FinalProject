#!/usr/bin/env python3

import math
import pickle
import sys
import time
import traceback
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import ME439_Robot

SIMULATE = False

try:
    import MotorControlArchitecture_v03 as mc
    import ME439_Sensors
    from pololu_drv8835_rpi import motors
except ImportError:
    SIMULATE = True


class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

class __sim_settings:
    def __init__(self):
        self.stage = 0
        self.t_stage_start = 0
        self.t_current = self.t_stage_start

# location size
WORKSPACE_WIDTH = 1.22 # m
WORKSPACE_LENGTH = 2.12 # m

# grid resolution
NODE_GRID_WIDTH = 50
NODE_GRID_LENGTH = 100

# start position, (0, 0) is lower left corner
START_X = 0 # m
START_Y = 0 # m
START_THETA = 0 # 0 = straight up, radians?

# end position (0, 0) is upper right corner
END_X = 1.22 # m
END_Y = 2.12 # m

PIVOT_ANGULAR_SPEED = 0.5
LINEAR_SPEED = 0.5

# scaling factors
ENDPOINT_SCALING_FACTOR = 4
REPULSION_SCALING_FACTOR = 0.125

# depending on the obstacles, we can get vectors with small or magnitudes.
# to keep things moving, we require a minimum magnitude
MINIMUM_VECTOR_MAGNITUDE = 0.1

MINIMUM_DISTANCE_THRESHOLD = 0.1

CORRECT_HEADING_THRESHOLD = np.radians(1)

# obstacle locations from lower left corner
OBSTACLES = [
    Obstacle(0.5, 0.5, 0.1),
    Obstacle(0.75, 1.5, 0.1)
]

def get_vector_from_current_position(x, y, vector_field):
    x_index = int((x / WORKSPACE_WIDTH) * vector_field.shape[1])
    y_index = int((y / WORKSPACE_LENGTH) * vector_field.shape[0])
    return vector_field[y_index, x_index]


def compute_stage_settings(robot, vector_field):
    stage_settings = [[1, 0, 0]]
    curr_x = START_X
    curr_y = START_Y
    curr_theta = START_THETA

    try:
        goal_reached = False
        while not goal_reached:
            curr_vector = get_vector_from_current_position(curr_x, curr_y, vector_field)
            grid_heading = np.arctan2(curr_vector[1], curr_vector[0])
            delta_heading = grid_heading - (np.pi / 2 + curr_theta)

            if np.abs(delta_heading) >= CORRECT_HEADING_THRESHOLD:
                stage_settings.append(robot.plan_pivot(PIVOT_ANGULAR_SPEED, delta_heading))

            # update internal heading
            curr_theta += delta_heading

            # go in a straight line
            distance = 0.05
            stage_settings.append(robot.plan_line(LINEAR_SPEED, distance))

            # update predicted x, y
            curr_x += np.cos(np.pi / 2 + curr_theta) * distance
            curr_y += np.sin(np.pi / 2 + curr_theta) * distance

            dist_away_from_goal = np.sqrt((curr_x - END_X) ** 2 + (curr_y - END_Y) ** 2)

            # check distance
            if dist_away_from_goal <= MINIMUM_DISTANCE_THRESHOLD:
                goal_reached = True

            print("X: {} | Y: {} | Theta: {} | Grid: {} | Delta H: {} | Distance: {}".format(curr_x, curr_y, np.degrees(curr_theta), np.degrees(grid_heading), np.degrees(delta_heading), dist_away_from_goal))
    except:
        print('exception!')
    
    stage_settings.append([1, 0, 0])
    return np.array(stage_settings)


def run_stage_settings_only(robot, stage_settings, fig):
    # plot the stage settings #
    sim_obj = __sim_settings()

    def update_drawing(num, rbt, rbtline, pathline, sim_obj):
        rbt.update_state()
        rbtline.set_data(rbt.corners_world_x, rbt.corners_world_y)
        pathline.set_data(rbt.path_world_x, rbt.path_world_y)

        sim_obj.t_current += robot.dt
        if (sim_obj.t_current - sim_obj.t_stage_start) >= stage_settings[sim_obj.stage, 0]:
            sim_obj.stage += 1

            # all stages done
            if sim_obj.stage >= stage_settings.shape[0]:
                sim_obj.stage = stage_settings.shape[0] - 1
                rbt.set_wheel_speed(0, 0)

            sim_obj.t_stage_start = sim_obj.t_current # reset the stage start time to present

        # Set the wheel speeds to the current stage, whatever it is
        rbt.set_wheel_speed(stage_settings[sim_obj.stage, 1], stage_settings[sim_obj.stage, 2])
        return rbtline, pathline

    robot0line, = plt.plot([], [], 'r-')
    robot0path, = plt.plot([], [], 'b--')
    plt.axis('equal')
    plt.xlim([0, WORKSPACE_WIDTH])
    plt.ylim([0, WORKSPACE_LENGTH])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Computed Path from Vector Field')
    line_ani = animation.FuncAnimation(
        fig, update_drawing, frames=1,
        fargs=(robot, robot0line, robot0path, sim_obj),
        interval=(robot.dt * 100), repeat=True, blit=False)
    try:
        plt.show()
    except:
        pass

def run_simulation(robot, vector_field, fig):
    robot.set_position(START_X, START_Y, START_THETA)
    stage_settings = compute_stage_settings(robot, vector_field)
    print(stage_settings)
    run_stage_settings_only(robot, stage_settings, fig)

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
            if (math.sqrt((x_loc - obstacle.x) ** 2 + (y_loc - obstacle.y) ** 2)) < obstacle.radius:
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
        resultant_mag = np.linalg.norm(resultant_vector) 
        if resultant_mag < MINIMUM_VECTOR_MAGNITUDE and not resultant_vector.any():
            resultant_vector /= resultant_mag
            resultant_vector *= MINIMUM_VECTOR_MAGNITUDE

        node_vectors[i, j] = resultant_vector

    # normalize node_vectors such that the maximum vector is of magnitude 1 #
    max_node_vector_mag = np.amax([[np.linalg.norm(col) for col in row] for row in node_vectors])
    node_vectors /= max_node_vector_mag

    # pull x/y components out for plotting
    vector_field_x_comp = node_vectors[:, :, 0]
    vector_field_y_comp = node_vectors[:, :, 1]

    fig = plt.figure()
    plt.quiver(x_nodes, y_nodes, vector_field_x_comp, vector_field_y_comp, units='width')

    # now set up a Robot object
    wheel_diameter = 0.0596 # m
    wheel_width = 0.133 # m
    body_length = 0.155 # m
    dt = 0.01
    robot0 = ME439_Robot.robot(wheel_width, body_length, dt)

    # simulation #
    if SIMULATE or (len(sys.argv) >= 2 and sys.argv[1] == 'sim'):
        run_simulation(robot0, node_vectors, fig)
        return

    # the real thing #
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
        stage_settings = np.array([[1,0,0],robot0.plan_line(0.2, 0.5),robot0.plan_pivot(-1,-np.pi),robot0.plan_line(0.2, 0.5),robot0.plan_pivot(1,np.pi)])

        stage = 0   # initialize in the 0th stage  
        active_controller_left.set_target(stage_settings[stage, 1])
        active_controller_right.set_target(stage_settings[stage, 2])

        t_stage_start = time.clock() # time.perf_counter() will give a reliable elapsed time. 
        
        # loop to continually execute the controller: 
        while True:
            # Maybe do things like checking the time or the displacement and updating the settings. 

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

    except Exception: 
        traceback.print_exc()
        try:
            motors.motor1.setSpeed(0)
            motors.motor2.setSpeed(0)
        except Exception:
            pass

if __name__ == '__main__':
    main()
