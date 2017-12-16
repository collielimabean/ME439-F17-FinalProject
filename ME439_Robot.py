#!/usr/bin/env python3

# Animation example code

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import math

   
class robot:
    def __init__(self, wheel_width, body_length, dt):
        self.dt = dt

        self.path_world_x = np.array([0])
        self.path_world_y = np.array([0])
        
        # history of the Encoder displacement readings
        self.left_encoder_history = np.array([0])
        self.right_encoder_history = np.array([0])
        
        # estimated state from Dead Reckoning
        self.x_estimated = 0
        self.y_estimated = 0
        self.theta_estimated = 0
        
        # estimated world angle from an arc center point (prior time step)
        self.estimated_psi_world_last = []
        
        # History of the x, y, theta estimates
        self.estimated_xytheta_history = np.array([[0,0,0]])
        
        self.wheel_width = wheel_width
        self.body_length = body_length

        self.set_position(0,0,0)
        self.set_velocity_bf(0,0)
        self.set_wheel_speed(0,0)  # left and right wheel speeds
        body_width_ratio = 0.8
        wheel_length_ratio = 0.4
        
        self.corners_bodyfixed_x = np.array( (wheel_width/2, wheel_width/2, body_width_ratio*wheel_width/2, body_width_ratio*wheel_width/2, -body_width_ratio*wheel_width/2, -body_width_ratio*wheel_width/2, -wheel_width/2, -wheel_width/2, wheel_width/2) )  # last corner closes the shape
        self.corners_bodyfixed_y = np.array( (body_length*(-wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(1-wheel_length_ratio/2), body_length*(1-wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(wheel_length_ratio/2), body_length*(-wheel_length_ratio/2), body_length*(-wheel_length_ratio/2)) )
        self.corners_bodyfixed_xy = np.vstack( (self.corners_bodyfixed_x,self.corners_bodyfixed_y) )
        
        self.get_corners_world_xy()  # get the Earth frame XY coords of the corners. 
        
        
    def set_position(self,x_center_world,y_center_world,theta):
        self.x_center_world = float(x_center_world)
        self.y_center_world = float(y_center_world)
        self.theta = float(theta)   
        
        self.path_world_x = np.append(self.path_world_x, self.x_center_world)
        self.path_world_y = np.append(self.path_world_y, self.y_center_world)
        
    def set_velocity_bf(self,ydot_center_bf,thetadot):
        self.xdot_ydot_center_bf = np.array([0, float(ydot_center_bf)])
        self.thetadot = float(thetadot)  
        
        self.set_velocity_world_from_bf()
        
    def set_velocity_world_from_bf(self):
        self.get_rotmat_body_to_world()
        xdot_ydot_center_world = np.dot( self.rotmat_body_to_world, self.xdot_ydot_center_bf )
        self.xdot_center_world = xdot_ydot_center_world[0]
        self.ydot_center_world = xdot_ydot_center_world[1]

    def set_velocity_world(self,xdot_center_world,ydot_center_world,thetadot):
        self.xdot_center_world = float(xdot_center_world)
        self.ydot_center_world = float(ydot_center_world)
        self.thetadot = float(thetadot)        
        
    def get_rotmat_body_to_world(self):
        self.rotmat_body_to_world = np.array([ [np.cos(self.theta), -np.sin(self.theta)],[np.sin(self.theta), np.cos(self.theta)]])
        
    def get_corners_world_xy(self): 
        self.get_rotmat_body_to_world()
        
        self.corners_rotated_xy= np.dot( self.rotmat_body_to_world, self.corners_bodyfixed_xy )
        self.corners_world_x = self.x_center_world + self.corners_rotated_xy[0]
        self.corners_world_y = self.y_center_world + self.corners_rotated_xy[1]
        self.corners_world_xy = np.vstack( (self.corners_world_x,self.corners_world_y) )
        
    def set_wheel_speed(self, left_wheel_speed, right_wheel_speed):
        self.left_wheel_speed = left_wheel_speed
        self.right_wheel_speed = right_wheel_speed
        
        # put in Equations (i) and (ii) here based on self.left_wheel_speed, self.right_wheel_speed, and self.wheel_width
        v_center_y_bf = (self.left_wheel_speed + self.right_wheel_speed)/2
        omega = (self.right_wheel_speed - self.left_wheel_speed)/self.wheel_width
        
        self.set_velocity_bf(v_center_y_bf, omega)
        
        
    def plan_line(self, speed, length):
        # correct sign mismatches between speed and length
        speed = np.abs(speed)*np.sign(length)
        lft_whl_spd = speed
        rgt_whl_spd = speed
        duration = length/speed
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        
    
    def plan_arc(self, radius, speed, angle):
        # arguments: radius (m), speed (m/s), angle (rad)
        lft_whl_spd = speed*(1 - (self.wheel_width/(2*radius) ) )
        rgt_whl_spd = speed*(1 + (self.wheel_width/(2*radius) ) )
        omega = speed/radius
        # correct funky combinations of signs
        omega = np.abs(omega)*np.sign(angle)
        duration = angle/omega
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        
        
    def plan_pivot(self, omega, angle):
        # arguments: angular velocity (rad/sec), angle to turn (rad)
        # returns duration, left_wheel_speed, right_wheel_speed
        # correct sign mismatches between omega and angle
        omega = np.abs(omega)*np.sign(angle)
        # compute wheel speeds
        radius = 0  # for a Pivot
        lft_whl_spd = omega*(radius-(self.wheel_width/2))
        rgt_whl_spd = omega*(radius+(self.wheel_width/2))
        duration = angle/omega
        
        return [duration, lft_whl_spd, rgt_whl_spd]
        

    def simulate_encoders(self, dt):
        left_encoder_value = self.left_encoder_history[-1] + self.left_wheel_speed * dt
        right_encoder_value = self.right_encoder_history[-1] + self.right_wheel_speed * dt
        self.update_encoders(left_encoder_value, right_encoder_value)        


    def update_encoders(self, left_encoder_value, right_encoder_value):
        self.left_encoder_history = np.append(self.left_encoder_history, left_encoder_value)     # [-1] syntax means the last element in the list
        self.right_encoder_history = np.append(self.right_encoder_history, right_encoder_value)    # [-1] syntax means the last element in the list
        
        
    def dead_reckoning(self):
        diff_left_displacement = np.diff(self.left_encoder_history[-2:])[0]
        diff_right_displacement = np.diff(self.right_encoder_history[-2:])[0]
        
        diff_pathlength = (diff_left_displacement+diff_right_displacement)/2
        diff_theta = (diff_right_displacement-diff_left_displacement)/self.wheel_width

        theta_avg = self.theta_estimated + diff_theta/2
        self.x_estimated += diff_pathlength * -np.sin(theta_avg)
        self.y_estimated += diff_pathlength * np.cos(theta_avg)
        self.theta_estimated += diff_theta
        
        self.estimated_xytheta_history = np.append( self.estimated_xytheta_history,[[self.x_estimated, self.y_estimated, self.theta_estimated]] , axis=0  ) 
        
        
    def set_estimated_position(self,x_center_world,y_center_world,theta):
        self.x_estimated = float(x_center_world)
        self.y_estimated = float(y_center_world)
        self.theta_estimated = float(theta)   
        
        self.estimated_xytheta_history = np.append( self.estimated_xytheta_history,[[self.x_estimated, self.y_estimated, self.theta_estimated]] , axis=0  ) 
        

    def compute_path_tracking_variables(self, path_segment_spec): 
        path_segment_Origin = np.array(path_segment_spec[0:2])
        # angle of the initial forward (y) direction of the path, rel to the World +Y axis
        path_segment_Theta0 = path_segment_spec[2]
        path_segment_Rsigned = path_segment_spec[3]
        path_segment_Length = path_segment_spec[4]

        # Forward distance relative to a Line path is computed along the Forward axis. 
        path_segment_y0_vector = np.array([-np.sin(path_segment_Theta0), np.cos(path_segment_Theta0)])
        # local X is computed perpendicular to the segment
        path_segment_x0_vector = np.array([-np.sin(path_segment_Theta0 - np.pi/2), np.cos(path_segment_Theta0 - np.pi/2)])
        path_segment_curvature = 1/path_segment_Rsigned
        
        # first handle the case of a Line
        if np.isinf(path_segment_Rsigned):
            path_segment_endpoint = path_segment_Origin + path_segment_Length*path_segment_y0_vector
            
            # compute position relative to Segment: 
            estimated_xy_rel_to_segment_origin = (self.estimated_xytheta_history[-1,0:2] - path_segment_Origin)
            estimated_segment_forward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_y0_vector)
            # The fraction completion can be estimated as the path length the robot has gone through, as a fraction of total path length on this segment
            estimated_segment_completion_fraction = estimated_segment_forward_pos / path_segment_Length

            estimated_segment_rightward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_x0_vector)
            
            estimated_y_local = 0   # y_local = 0 by definition: Local coords are defined relative to the closest point on the path. 
            estimated_x_local = estimated_segment_rightward_pos
            estimated_theta_local = self.estimated_xytheta_history[-1,2] - path_segment_Theta0 
        
        else:
            curve_sign = np.sign(path_segment_Rsigned)
            path_segment_circle_center = path_segment_Origin + path_segment_Rsigned*-path_segment_x0_vector
            # determine the angular displacement of this arc. SIGNED quantity! 
            path_segment_angular_displacement = path_segment_Length/path_segment_Rsigned
            path_segment_ThetaEnd = path_segment_Theta0 + path_segment_angular_displacement
            estimated_xy_rel_to_circle_center = (self.estimated_xytheta_history[-1,0:2] - path_segment_circle_center)
            # Compute angle of a vector from circle center to Robot, in the world frame, relative to the +Yworld axis. 
            # Note how this definition affects the signs of the arguments to "arctan2"
            estimated_psi_world = np.arctan2(-estimated_xy_rel_to_circle_center[0], estimated_xy_rel_to_circle_center[1])
            # unwrap the angular displacement
            if not self.estimated_psi_world_last:
                self.estimated_psi_world_last = estimated_psi_world
            while estimated_psi_world - self.estimated_psi_world_last > np.pi: # was negative, is now positive --> should be more negative. 
                estimated_psi_world += -2*np.pi
            while estimated_psi_world - self.estimated_psi_world_last < -np.pi: # was positive and is now negative --> should be more positive. 
                estimated_psi_world += 2*np.pi
            # update the "last angle" estimate. 
            self.estimated_psi_world_last = estimated_psi_world
            # The local path forward direction is perpendicular (clockwise) to this World frame origin-to-robot angle. 
            estimated_path_theta = estimated_psi_world + np.pi/2*curve_sign
            # The fraction completion can be estimated as the path angle the robot has gone through, as a fraction of total angular displacement on this segment
            estimated_segment_completion_fraction = (estimated_path_theta - path_segment_Theta0) / path_segment_angular_displacement

            estimated_y_local = 0  # by definition of local coords
            # x_local is positive Inside the circle for Right turns, and Outside the circle for Left turns
            estimated_x_local = curve_sign*(np.sqrt(np.sum(np.square(estimated_xy_rel_to_circle_center))) - np.abs(path_segment_Rsigned) ) 
            estimated_theta_local = self.estimated_xytheta_history[-1,2] - estimated_path_theta

        estimated_theta_local = fix_angle_pi_to_neg_pi(estimated_theta_local)
        self.estimated_x_local = estimated_x_local
        self.estimated_theta_local = estimated_theta_local
        self.path_segment_curvature = path_segment_curvature
        self.estimated_segment_completion_fraction = estimated_segment_completion_fraction
        
    def set_wheel_speeds_for_path_tracking(self, Vmax, Beta, gamma):
        # parameters for the controller are Maximum allowable speed, and 
        # controller gains:
        #   Beta (gain on lateral error, mapping to lateral speed)
        #   gamma (gain on heading error, mapping to rotational speed)
        theta_local = self.estimated_theta_local
        x_local = self.estimated_x_local
        curvature = self.path_segment_curvature
        
        # First set the speed with which we want the robot to approach the path
        xdot_local_desired = -Beta*x_local
        # limit it to +-Vmax
        xdot_local_desired = np.min([np.abs(xdot_local_desired),abs(Vmax)])*np.sign(xdot_local_desired)
        
        # Next set the desired theta_local 
        theta_local_desired = -np.arcsin(xdot_local_desired/Vmax)
                
        # Next set speed to fall to zero if the robot is more than 90 deg from the heading we want it to have
        Vc = Vmax * np.cos(fix_angle_pi_to_neg_pi(theta_local_desired - theta_local))
        Vc = np.max([Vc,0])
        
        # Finally set the desired angular rate
        omega = gamma*fix_angle_pi_to_neg_pi(theta_local_desired - theta_local) + Vc*(curvature/(1+curvature*x_local))*np.cos(theta_local)
        
        self.set_wheel_speeds_from_robot_velocities(Vc, omega)
        
        
    def set_wheel_speeds_from_robot_velocities(self, forward_velocity, angular_velocity):
        right_wheel_speed = forward_velocity + angular_velocity*self.wheel_width/2 
        left_wheel_speed = forward_velocity - angular_velocity*self.wheel_width/2 
        
        self.set_wheel_speed(left_wheel_speed, right_wheel_speed)
        
        
    def update_state(self):
        # *****
        # Program some rules for updating the state: 
        # *****
        self.set_velocity_world_from_bf()  # convert the body-frame velocity of the robot into world-frame coordinates. 
        
        self.x_center_world += self.dt * self.xdot_center_world   # simulates constant velocity toward +x
        self.y_center_world += self.dt * self.ydot_center_world   # simulates constant velocity toward +y
        self.theta += self.dt * self.thetadot      # simulates constant rotational speed (+z rotation)
        
        # Encoder values update
        self.simulate_encoders(self.dt)
        self.dead_reckoning()
        
        # this updates the path. 
        self.path_world_x = np.append(self.path_world_x, self.x_center_world)
        self.path_world_y = np.append(self.path_world_y, self.y_center_world)
        
        self.get_corners_world_xy()


#%% General Functions        
def fix_angle_pi_to_neg_pi(angle): 
    return np.mod(angle+np.pi,2*np.pi)-np.pi        


def convert_stage_settings_to_path_specs(xytheta_init, stages, wheel_width):
    xytheta_beginning = xytheta_init
    paths = np.array([[]])
    for ii in range(0,stages.shape[0]):
        xy_beg = xytheta_beginning[0:2]
        theta_beg = xytheta_beginning[2]
        delta_time = stages[ii,0]
        left_vel = stages[ii,1]
        right_vel = stages[ii,2]
        robot_vel = (left_vel+right_vel)/2
        robot_omega = (right_vel-left_vel)/wheel_width
        path_length = delta_time*robot_vel
        delta_theta = delta_time*robot_omega
        if robot_omega == 0:
            path_radius = np.inf
        else:
            path_radius = robot_vel/robot_omega
            
        theta_end = theta_beg+delta_theta 

        # handle the case of a Line
        if np.isinf(path_radius):
            xy_end = xy_beg + path_length*np.array([-np.sin(theta_beg), np.cos(theta_beg)])
        else:
            # find circle center. Note it is a distance of (-R) out the wheel axis to the Right of the robot. 
            circle_center = xytheta_beginning[0:2] + -path_radius*np.array([np.cos(theta_beg), np.sin(theta_beg)])
            xy_end = circle_center + path_radius*np.array([np.cos(theta_end), np.sin(theta_end)])
        
        if not (xy_end == xy_beg).all() :
            if not paths.any():
                paths = np.atleast_2d(np.append(xytheta_beginning,[path_radius,path_length]))
            else:
                paths = np.vstack((paths,np.append(xytheta_beginning,[path_radius,path_length])))
        
        xytheta_beginning = np.append(xy_end,theta_end)
        
    return paths
    
        
def update_drawing(num, rbt, rbtline, pathline) :
    global segment, Vmax, Beta, gamma
    
    try:
        # Check the path progress and set the robot controls accordingly
        rbt.compute_path_tracking_variables(path_specs[segment,:])
        # check if old segment is done. If so, switch to next segment. 
        if rbt.estimated_segment_completion_fraction > 1.0:
            segment += 1    
            # Clear the memory of the prior angle estimate. It causes a wrapping problem if it persists across segments. 
            rbt.estimated_psi_world_last = None  
            # when path is complete, segment will exceed the length of the path_specs array and this next line will throw an Exception. Then the animation will be over. 
            if segment >= path_specs.shape[0]:  # throw an exception if the new segment doesn't exist
                raise ValueError("Path is Complete")
            rbt.compute_path_tracking_variables(path_specs[segment])
        rbt.set_wheel_speeds_for_path_tracking(Vmax, Beta, gamma)
        
        rbt.update_state()
        rbtline.set_data(rbt.corners_world_x,rbt.corners_world_y)
        pathline.set_data(rbt.path_world_x, rbt.path_world_y)
        
    except ValueError: 
        rbt_path = plot_robot_path(rbt)
        raise
    
    return rbtline, pathline

def simulate_robot():
    # Set up the Animation plot
    fig1= plt.figure()
    
    robot0line, = plt.plot([], [], 'r-')
    robot0path, = plt.plot([],[], 'b--')
    plt.axis('equal')   # Note for some reason this has to come before the "xlim" and "ylim" or an explicit axis limits command "plt.axis([xmin,xmax,ymin,ymax])"
    plt.xlim(-4, 4)
    plt.ylim(-4, 4)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Robot Test')
    
    global dt, t_current, t_stage_start
    
    dt = 0.05   # seconds
    
    t_stage_start = 0  # time.perf_counter() if real-life   # Get the current performance counter time
    t_current = t_stage_start   
    line_ani = animation.FuncAnimation(fig1, update_drawing, frames=1, fargs=(robot0, robot0line, robot0path), interval= (dt*1000), repeat=True, blit=False)
    plt.show()
    return line_ani
    

    
def plot_robot_path(rbt):
    #%% Plot the resulting dead reckoning data
    f = plt.figure()
    reconstructed_path, = plt.plot(rbt.estimated_xytheta_history[:,0], rbt.estimated_xytheta_history[:,1])
    plt.axis('equal')
    plt.show()
    return reconstructed_path
