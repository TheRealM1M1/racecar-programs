"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: Team 6 - Ferrari Rochers

Purpose: Path planner for wall following << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys
import numpy as np
import math as math

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
speed = 0
angle = 0


########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed 
    global angle
    #rc.drive.set_max_speed(0.5)
    speed = 1
    angle = 0

# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global speed 
    global angle 

    scan = rc.lidar.get_samples() # collect Lidar scan 

    sweep_range_deg = 120         # Total sweep range (-60° to +60°)
    triangle_span_deg = 30        # Width of each triangle
    triangle_depth = 100          # How far out to check (meters)
    min_clearance = 150           # Required clearance inside triangle
    angle_step = 2    

    best_direction = 0
    max_clearance = 0

    required_clearance = 120

    for center_angle in range(-75, 76, angle_step):
        start_angle = center_angle - triangle_span_deg // 2
        end_angle = center_angle + triangle_span_deg // 2
        

        # Sample LiDAR points inside triangle
        inside = []
        for angle in range(start_angle, end_angle + 1):
            dist = rc_utils.get_lidar_average_distance(scan, calc_angle(angle))
            if dist > triangle_depth:
                inside.append(dist)

        # Check clearance
        if len(inside) == 0 or min(inside) < required_clearance:
            continue
        min_dist = min(inside)
        
        if min_dist > max_clearance:
            best_direction = center_angle
            max_clearance = min_dist

    right = rc_utils.get_lidar_average_distance(scan, 60)    
    left = rc_utils.get_lidar_average_distance(scan, 300) 
    
    
    
    steering = best_direction / 70.0
    # if right < 20:
    #     steering += -0.2
    # elif left < 20:
    #     steering += 0.2
    speed = 1 if max_clearance > 220 else 0.6
    steering = rc_utils.clamp(steering, -1.0, 1.0)

    print(f"Angle: {best_direction} Clearance: {max_clearance:.2f} Speed: {speed:.2f} Steering: {steering:.2f}")
    rc.drive.set_speed_angle(speed, steering)



# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    pass # Remove 'pass and write your source code for the update_slow() function here


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
