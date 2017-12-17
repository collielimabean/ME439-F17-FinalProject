ME439 Fall 2017 Final Project: Vector Field Potential Path Planning
===================================================================

Alex Boulanger, Alex Dixon, William Jen

## Introduction
The vector field potential is a path-planning algorithm. In other words,
given a map of all the obstacles, create a path that will get the robot from 
point A to point B. This is done by giving the end point B a "pull" and each
obstacle a "repelling" force.

## Running
* Simulation: `sudo python3 VectorField.py sim`.
* Actual hardware: `sudo python3 VectorField.py`

You may also omit the "sim" argument if you are not on the RPi - the program will detect if the hardware libraries are not available and run the simulation instead.

## Parameters
This vector field algorithm has multiple parameters with which to change the 
behavior. They are specified below.

* Robot dimensions (`WHEEL_WIDTH` and `BODY_LENGTH`)
* Timestep (`DELTA_T`)
* Workspace dimensions (`WORKSPACE_WIDTH` and `WORKSPACE_LENGTH`)
* Vector field resolution (`NODE_GRID_LENGTH` and `NODE_GRID_WIDTH`)
* Vector field scaling factors (`ENDPOINT_SCALING_FACTOR`, `REPULSION_SCALING_FACTOR`):
	These control the magnitude of the obstacle repulsion and the
	endpoint pull vector.
* Minimum vector field magnitude (`MINIMUM_VECTOR_MAGNITUDE`): Some
	vectors may be very small due to the obstacle and end point vectors
	cancelling to some degree. This value forces those small vectors
	to have the specified minimum magnitude.
* Initial robot pose (`START_X`, `START_Y`, `START_THETA`). 
	Note that `START_THETA`is referenced off the +y axis (i.e. 0 is
	straight up)
* End robot position (`END_X`, `END_Y`)
* Robot speeds (`PIVOT_ANGULAR_SPEED`, `LINEAR_SPEED`)
* End point tolerance (`MINIMUM_DISTANCE_THRESHOLD`) 
* Robot headling alignment tolerance (`CORRECT_HEADING_THRESHOLD`):
	To reduce motion jerkiness, this angle tells the robot when to pivot
	to correct its heading
* Obstacle list (`OBSTACLES`)

