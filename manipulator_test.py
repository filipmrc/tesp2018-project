# -*- coding: utf-8 -*-
import os, inspect
import numpy as np
import pybullet as p
import time
import pybullet_data
from manipulator import Manipulator
from audio_lib import Sine

grip = 0

def update_keys():
    global grip
    keys = p.getKeyboardEvents()
    result = np.array([0,0,0])
    for k in keys:
        if k == ord('l'):
            result = np.array([0.005,0,0])
        elif k == ord('j'):
            result = np.array([-0.005, 0, 0])
        elif k == ord('i'):
            result = np.array([0, 0.005, 0])
        elif k == ord('k'):
            result = np.array([0, -0.005, 0])
        elif k == ord('o'):
            result = np.array([0, 0, 0.005])
        elif k == ord('u'):
            result = np.array([0, 0, -0.005])
        elif k == ord('u'):
            result = np.array([0, 0, -0.005])

        if k == ord('b'):
            grip = 1
        elif k == ord('n'):
            grip = 0

    return result

def vibrate():
    pulse1 = Sine(frequency=80)
    pulse1.attack = 0.071
    pulse1.sustain = 0.479
    pulse1.release = 0.043
    pulse1.channel = 1
    #pulse1.play(1.0, blocking=True)

    pulse2 = Sine(frequency=80)
    pulse2.delay = 0.1
    pulse2.attack = 0.071
    pulse2.sustain = 0.479
    pulse2.release = 0.043
    pulse2.channel = 2

    pulse3 = pulse1 + pulse2
    pulse3.play(1.0, blocking=True)
        
cid = p.connect(p.GUI_SERVER)  # stuff above hangs much of the time for some reason
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) #get current directory

p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)

# Spawn the Jaco manipulator
armStartPos = [1,0,0.9]
armStartOrientation = p.getQuaternionFromEuler([0,0,0])
endEffectorIndex = 7
jaco = [p.loadURDF(currentdir+"/models/urdf/jaco.urdf")] #load arm
p.resetBasePositionAndOrientation(jaco[0],armStartPos,armStartOrientation)

# Spawn environment
plane = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)] #load plane
objects = [p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)]
objects = [p.loadURDF("cube_small.urdf", 0.950000, -0.100000, 0.700000, 0.000000, 0.000000, 0.707107, 0.707107)]
objects = [p.loadURDF("sphere_small.urdf", 0.850000, -0.400000, 0.700000, 0.000000, 0.000000, 0.707107, 0.707107)]
objects = [p.loadURDF("duck_vhacd.urdf", 0.850000, -0.400000, 0.900000, 0.000000, 0.000000, 0.707107, 0.707107)]

# Initialize the manipulator class
manipulator = Manipulator(jaco,cid,endEffectorIndex,'p', [6,7,8])
jointPositions=[ 0, 3.14, 3.14/2, -3.14/2, -3.14/2, 0, 0, 0, 0] #set joint position goal
manipulator.set_joint_position_goal(jointPositions)
manipulator.update() #update joint position
time.sleep(1)
pose = manipulator.get_end_effector_pose()
goal_pos = pose[0:3]
goal_rot = pose[3:7]

while(1):

    # Check key press
    cmd = update_keys()

    # Update pose goal
    goal_pos = goal_pos + cmd
    manipulator.set_frame_pose_goal(endEffectorIndex,goal_pos,goal_rot)

    # Check gripper flag and update gripper goal
    if grip==0:
        manipulator.open_gripper()
    elif grip==1:
        manipulator.close_gripper()

    # Update manipulator state
    manipulator.update()

    # Check for contact and vibrate
    c = manipulator.check_contact()
    if c:
        print("vibrate")
        vibrate()

    time.sleep(0.01)


p.disconnect()