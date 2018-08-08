#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os, inspect
import numpy as np
import pybullet as p
import time
import pybullet_data
from manipulator import Manipulator
from geometry_msgs.msg import Pose
from flow_sim import Natgorithm


class HapticDemo(object):
    def __init__(self,mnp):
        # Pass manipulator hook
        self.m = mnp

        # Initialize ROS node
        rospy.init_node('listener', anonymous=True)

        # Start up the ROS subscriber for Kinect
        rospy.Subscriber("kinect", Pose, self.callback)

        # Start spinning thread
        rospy.spin()


    def callback(self, data):
        # Get hand position relative to starting point
        # (i.e. delta to be applied to current manipulator position)
        delta = np.array([data.position.x,data.position.y,data.position.z])

        # Get current end-effector pose, sparate position and rotation
        pose = manipulator.get_end_effector_pose()
        goal_pos = pose[0:3] + delta
        goal_rot = pose[4:7]

        # Set the goal for the manipulator object and
        self.m.set_frame_pose_goal(endEffectorIndex, goal_pos, goal_rot)
        self.m.update()

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
    endEffectorIndex = 7
    jaco = [p.loadURDF(currentdir + "/models/urdf/jaco.urdf")]  # load arm
    p.resetBasePositionAndOrientation(jaco[0], armStartPos, armStartOrientation)

    # Spawn environment
    plane = [
        p.loadURDF("plane.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)]  # load plane
    objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]

    # Initialize the manipulator class
    manipulator = Manipulator(jaco, cid, endEffectorIndex, 'p')
    jointPositions = [0, 3.14, 3.14 / 2, -3.14 / 2, -3.14 / 2, 0, 0, 0, 0]  # set joint position goal
    manipulator.set_joint_position_goal(jointPositions)
    manipulator.update()  # update joint position
    time.sleep(1)

    # Initialize haptic demo class/container whatever
    n = HapticDemo(manipulator)