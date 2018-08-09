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

        self.pulse1 = Sine(frequency=160)
        self.pulse1.attack = 0.015
        self.pulse1.sustain = 0.02
        self.pulse1.release = 0.004
        self.pulse1.channel = 1
        self.pulse2 = self.pulse1 * 1.0
        self.pulse2.channel = 2
        self.pulse3 = self.pulse1 + self.pulse2
        self.pulse3 *= 5

        self.noise1 = Sine(frequency=100)
        self.noise1.channel = 1
        self.noise2 = Sine(frequency=97)
        self.noise2.channel = 2
        self.noise3 = self.noise1 + self.noise2
        self.noise3 *= 0.1

        self.pulse3 += self.noise3

        self.touched_last = False
        self.touched_last_t = time.time()

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
            delta = np.array([(data.position.z - self.kinect_zero[2])/500, (data.position.x- self.kinect_zero[0])/500, (data.position.y- self.kinect_zero[1])/500])

            # Get current end-effector pose, sparate position and rotation
            goal_pos = self.zero_pose[0:3] + delta
            goal_rot = self.zero_pose[4:7]

            # Check for collisions
            c = manipulator.check_contact()
            if c and time.time() - self.touched_last_t > 0.7:
                if not self.touched_last:
                    self.pulse3.play(blocking=False, loop=False, device=3)
                else:
                    self.noise3.play(blocking=False, loop=True, device=3)
                self.touched_last_t = time.time()
            elif not c and self.touched_last:
                self.noise3.stop()
                self.pulse3.stop()
            self.touched_last = c

            # Set the goal for the manipulator object and
            self.m.set_frame_pose_goal(endEffectorIndex, goal_pos, goal_rot)
            self.m.update()


if __name__ == '__main__':
    # Set up the simulation
    cid = p.connect(p.GUI_SERVER)  # stuff above hangs much of the time for some reason

    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # I hate this block of code
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(1)

    # Get the current directory
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))  # get current directory

    # Spawn the Jaco manipulator
    armStartPos = [1, 0, 0.8]
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
    jointPositions = [0, 3.14, 3.14 / 2, -3.14 / 2, 0, 0, 0, 0, 0]  # set joint position goal
    manipulator.set_joint_position_goal(jointPositions)
    manipulator.update()  # update joint position
    time.sleep(1)

    # Initialize haptic demo class/container whatever
    n = HapticDemo(manipulator)
