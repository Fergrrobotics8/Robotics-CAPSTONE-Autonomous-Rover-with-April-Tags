#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
#from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController
#from path_planner import dijkstras

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        #self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements() #type list (matriz de 1x3 meas[0][2], posicion del tag [[x, y, z, id]])
        imu_meas = self.ros_interface.get_imu()

        if meas!=None:
            # convert meas into a np array
            measu = meas[0]

            # IMPORTANTE USAR ***MEASU*** Y NO MEAS!!!

            #print measurements, python 2 btw...
            print("*******")
            print("Tag: "),
            print(measu[3])
            measu3f=np.round(measu, 3)
            print("Measurements: "),
            print(measu3f[:3])

            #print state
            state = np.array([0.0, 0.0, 0.0])
            print("State: "),
            print(state)
            
            goal = np.array([measu[0], -measu[1], measu[2]])

            vw = self.diff_drive_controller.compute_vel(state, goal)

            #print comanded velocity
            vw3f=np.round(vw, 3)
            print("Computed command vel: "),
            print(vw3f[:2])

            if vw[2] == False:
                self.ros_interface.command_velocity(vw[0], vw[1])
            if vw[2] == True:
                print("*****The robot arrived its destination*****")
                print()
        else:
            #self.ros_interface.command_velocity(0, 0)
            return
    
        
           
def main(args):
    rospy.init_node('robot_control')
    print("")
    print("****ROBOT STARTED****")

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    #This was added to have the robot run for one second.
    #time=rospy.get_time()
    while not rospy.is_shutdown(): #and rospy.get_time()-time < 1: #remove and stmt
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass