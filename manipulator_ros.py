#!/usr/bin/env python
import rospy
import os, inspect
import numpy as np
import pybullet as p
import time
import pybullet_data
from manipulator import Manipulator
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from audio_lib import Sine

class HapticDemo(object):
    def __init__(self,mnp):
        # Pass manipulator hook
        self.m = mnp

        # Initialize ROS node
        rospy.init_node('listener', anonymous=True)

        # Start up the ROS subscriber for Kinect
        rospy.Subscriber("/kinect_pose", Pose, self.callback)

        # Flag for Kinect initialization
        self.kinect_zero_set = False

        # Start spinning thread
        rospy.spin()


    def callback(self, data):
        if not self.kinect_zero_set:
            self.kinect_zero_set = rospy.get_param("kinect_zero_set")
            self.kinect_zero = np.array([data.position.x,data.position.y,data.position.z])

            self.zero_pose = self.m.get_end_effector_pose()
        else:
            # Get hand position relative to starting point
            # (i.e. delta to be applied to current manipulator position)
            delta = np.array([(data.position.x - self.kinect_zero[0])/1000, (data.position.y- self.kinect_zero[1])/1000, (data.position.z- self.kinect_zero[2])/1000])

            # Get current end-effector pose, sparate position and rotation
            goal_pos = self.zero_pose[0:3] + delta
            goal_rot = self.zero_pose[4:7]

            # Check for collisions
            collision = self.m.check_contact()

            # If in collision apply vibration
            if collision:
                self.vibrate()

            # Set the goal for the manipulator object and
            self.m.set_frame_pose_goal(endEffectorIndex, goal_pos, goal_rot)
            self.m.update()

    def vibrate(self):
        pulse1 = Sine(frequency=80)
        pulse1.attack = 0.071
        pulse1.sustain = 0.479
        pulse1.release = 0.043
        pulse1.channel = 1
        pulse1.play(1.0, blocking=True)

if __name__ == '__main__':
    # Set up the simulation
    cid = p.connect(p.GUI_SERVER)  # stuff above hangs much of the time for some reason

    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # I hate this block of code
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)

    # Get the current directory
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))  # get current directory

    # Spawn the Jaco manipulator
    armStartPos = [1, 0, 0.7]
    armStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    endEffectorIndex = 8
    jaco = [p.loadURDF(currentdir + "/models/urdf/jaco.urdf")]  # load arm
    p.resetBasePositionAndOrientation(jaco[0], armStartPos, armStartOrientation)

    # Spawn environment
    p.loadURDF("plane.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)  # load plane
    p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
    objects = [p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
    objects = [p.loadURDF("jenga/jenga.urdf", 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
    objects = [p.loadURDF("jenga/jenga.urdf", 1.100000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
    objects = [p.loadURDF("jenga/jenga.urdf", 1.000000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]
    objects = [p.loadURDF("jenga/jenga.urdf", 0.900000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

    # Initialize the manipulator class
    manipulator = Manipulator(jaco, cid, endEffectorIndex, 'p')
    jointPositions = [0, 3.14, 3.14 / 2, -3.14 / 2, -3.14 / 2, 0, 0, 0, 0]  # set joint position goal
    manipulator.set_joint_position_goal(jointPositions)
    manipulator.update()  # update joint position
    time.sleep(1)

    # Initialize haptic demo class/container whatever
    n = HapticDemo(manipulator)
