# -*- coding: utf-8 -*-
import os, inspect
import numpy as np
import pybullet as p
import time
import pybullet_data
from manipulator import Manipulator
from audio_lib import Sine


def update_keys():
    keys = p.getKeyboardEvents()
    result = np.array([0,0,0])
    for k in keys:
        if k == ord('l'):
            result = np.array([0.01,0,0])
        elif k == ord('j'):
            result = np.array([-0.01, 0, 0])
        elif k == ord('i'):
            result = np.array([0, 0.01, 0])
        elif k == ord('k'):
            result = np.array([0, -0.01, 0])
        elif k == ord('o'):
            result = np.array([0, 0, 0.01])
        elif k == ord('u'):
            result = np.array([0, 0, -0.01])
        elif k == ord('u'):
            result = np.array([0, 0, -0.01])

    return result

#def matprint(mat, fmt="g"):
#    col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
#    for x in mat:
#        for i, y in enumerate(x):
#            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end = " ")
#        print("")

def vibrate():
    pulse1 = Sine(frequency=80)
    pulse1.attack = 0.071
    pulse1.sustain = 0.479
    pulse1.release = 0.043
    pulse1.channel = 1
    pulse1.play(1.0, blocking=True)
        
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
    cmd = update_keys()
    goal_pos = goal_pos + cmd
    manipulator.set_frame_pose_goal(endEffectorIndex,goal_pos,goal_rot)
    manipulator.close_gripper()
    manipulator.update()
    c = manipulator.check_contact()
    if c:
        vibrate()
    #matprint(manipulator.get_manipulability_ellipsoid()) # 9,11,13 are gripper indices
    #print("---------------------------------")
    #goal_pos = goal_pos + np.array([0,0,-0.01])
    time.sleep(0.01)


p.disconnect()