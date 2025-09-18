#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
# import rospy

import yaml
import numpy as np

import sys

# TODO for student: Comment this section when running on the robot
from RobotSim import RobotSim
import matplotlib.pyplot as plt

plt.style.use('ggplot')


# TODO for student: uncomment when changing to the robot
# from ros_interface import ROSInterface

# TODO for student: User files, uncomment as completed
from ShortestPath import dijkstras
# from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """

    def __init__(self, world_map, occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing,
                 t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """

        # TODO for student: Comment this when running on the robot
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        # self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        self.markers   = world_map
        self.vel       = np.array([0.,0.])
        self.imu_meas  = np.array([]) 
        self.meas      = []
        self.max_speed = max_speed
        self.max_omega = max_omega
        
        
        # Uncomment as completed
        # self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and estimation
        are done. This function is called at 60Hz
        """
        # TODO for student: Comment this when running on the robot
        meas = self.robot_sim.get_measurements() # N by 5 numpy array (x,y,theta,id,time)
        imu_meas = self.robot_sim.get_imu() # numpy vector 5 (acc_x, acc_y, acc_z, omega, time)
        
        #print(imu_meas)
        
        # TODO INTEGRAR DJAKSTRA ALGORITMO Y CAMBIAR STATE CADA VEZ CON LAS OMEGAS GIROS VELOCIDADES ETC BOI https://www.youtube.com/watch?v=_lHSawdgXpI
        
        if meas != None and meas != []:
            #time
            time = round(float(imu_meas[4]), 3)
            print("Time: ", time)


            print("Active tags: ", len(meas))
            for i,tag in  enumerate(meas):
                print("Tag: ", int(meas[i][3]))
                print("Distance x,y and angule theta: ", np.round(tag[:3],3))


            measu=meas[0][:3]

            #print state
            state = np.array([0.0, 0.0, 0.0])
            print("State: "),
            print(state)


            print("*********")

            

            goal = np.array([measu[0], -measu[1], measu[2]])

            vw = self.diff_drive_controller.compute_vel(state, goal)

            #print comanded velocity
            vw3f=np.round(vw, 3)
            print("Computed command vel/ang: "),
            print(vw3f[:2])

            if vw[2] == False:
                self.robot_sim.command_velocity(vw[0], vw[1])
            if vw[2] == True:
                print("*****The robot arrived its destination*****")
        else:
            #self.robot_sim.command_velocity(0,0)
            return



        

        # TODO for student: Use this when transferring code to robot
        # meas = self.ros_interface.get_measurements()
        # imu_meas = self.ros_interface.get_imu()

        return



def main(args):
    # Load parameters from yaml
    param_path = 'params.yaml'  # rospy.get_param("~param_path")
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw, Loader=yaml.FullLoader)
    #params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Initialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # TODO for student: Comment this when running on the robot
    # Run the simulation
    

    while not robotControl.robot_sim.done and plt.get_fignums():
        robotControl.process_measurements()
        robotControl.robot_sim.update_frame()
        

    plt.ioff()
    plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    """r = rospy.Rate(60)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)"""


if __name__ == "__main__":
    main(sys.argv)
