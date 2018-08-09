import os, inspect
import numpy as np
import pybullet as p
import time
import pybullet_data


# from utils.general import *
# import utils.transformations as tf


class Manipulator(object):
    """
    Provides a pybullet API wrapper for simpler interfacing and manipulator-specific functions.
    The update() function should be called in a loop in order to store joint states and update joint controls.
    """

    def __init__(self, arm, cid, ee_link_index, control_method, gripper_indices=[]):
        # user selected parameters -- non-private can be modified on the fly
        self.arm = arm
        self.cid = cid
        self.num_joints = p.getNumJoints(self.arm[0])
        self.joint_infos = [p.getJointInfo(self.arm[0], i) for i in range(self.num_joints)]

        self._active_joint_indices = [j for j, i in zip(range(p.getNumJoints(self.arm[0])), self.joint_infos) if
                                      i[3] > -1]
        self._gripper_indices = gripper_indices
        self._control_method = control_method
        self._ee_link_index = ee_link_index

    # GET - PRIVATE
    # --------------------------------------------------------------------------------------------------------------

    def _get_joint_states(self):
        """
        Get positions, velocities and torques of active joints (as opposed to passive, fixed joints)
        """
        joint_states = p.getJointStates(self.arm[0], range(p.getNumJoints(self.arm[0])))
        joint_states = [j for j, i in zip(joint_states, self.joint_infos) if i[3] > -1]  # get only active states
        self.jnt_pos = [state[0] for state in joint_states]
        self.jnt_vel = [state[1] for state in joint_states]
        self.jnt_torq = [state[3] for state in joint_states]
        return self.jnt_pos, self.jnt_vel, self.jnt_torq

    def _get_link_state(self, link_index):
        """
        Returns information on the link URDF frame and centre of mass poses in the world frame
        """
        result = p.getLinkState(self.arm[0],
                                linkIndex=link_index)  # , computeLinkVelocity=1, computeForwardKinematics=1)
        return result

    def _get_link_pose(self, link_index):
        """
        Get a links pose in the world frame as a 7 dimensional vector containing the
        position (x,y,z) and quaternion (x,y,z,w)
        """
        result = self._get_link_state(link_index)
        link_frame_pos = np.asarray(result[4])
        link_frame_rot = np.asarray(result[5])
        link_frame_pose = np.concatenate((link_frame_pos, link_frame_rot))  # transform from x,y,z,w to w,x,y,z
        return link_frame_pose

    def _get_link_jacobian(self, link_index):
        """
        Get the Jacobian of a link frame in the form 6xN [J_trans; J_rot]
        """
        zero_vec = [0.0] * len(self.jnt_pos)
        jac_t, jac_r = p.calculateJacobian(self.arm[0], link_index, [0, 0, 0], self.jnt_pos, zero_vec, zero_vec)
        J = np.concatenate((jac_t, jac_r), axis=0)
        return J

    # SET AND CONTROL LOOPS - PRIVATE
    # --------------------------------------------------------------------------------------------------------------

    def _hard_set_joint_positions(self, cmd):
        """
        Set joint positions without simulating actual control loops
        """
        k = 0
        cmd_ind = [j for j, i in zip(range(p.getNumJoints(self.arm[0])), self.joint_infos) if i[3] > -1]
        for j in cmd_ind:
            p.resetJointState(self.arm[0], j, cmd[k])
            k = k + 1

    def _joint_position_control(self, cmd):
        """
        Position control loop, max torques set to an arbitrary 5000
        """
        p.setJointMotorControlArray(self.arm[0], jointIndices=self._active_joint_indices,
                                    controlMode=p.POSITION_CONTROL, targetPositions=cmd, forces = [100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000])

    def _joint_velocity_control(self, cmd):
        """
        Velocity control loop - not really tested
        """
        p.setJointMotorControlArray(self.arm[0], jointIndices=self._active_joint_indices,
                                    controlMode=p.VELOCITY_CONTROL, targetVelocities=cmd)

    # FETCH INFO FOR ANY NUMBER OF FRAMES
    # ----------------------------------------------------------------------------------------------------------------

    def get_states(self, link_indices):
        """
        Return a tuple of states of URDF links given link indices
        """
        states = ()
        for i in link_indices:
            link_state = self._get_link_state(i)
            states = states + (link_state,)
        return states

    def get_poses(self, link_indices):
        """
        Return poses of URDF link frames given link indices.
        Poses are returned in the form of a tuple of numpy arrays (np.array(x,y,z,w,x,y,z))
        """
        poses = ()
        for i in link_indices:
            link_pose = self._get_link_pose(i)
            poses = poses + (link_pose,)
        return poses

    def get_jacobians(self, link_indices):
        """
        Return Jacobians of frames given frame_indices in the form of numpy arrays
        """
        jacobians = ()
        for i in link_indices:
            link_jacobian = self._get_link_jacobian(i)
            jacobians = jacobians + (link_jacobian,)
        return jacobians

    def get_link_names(self):
        """
        Returns a list of all link names
        """
        names = []
        for info in self.joint_infos:
            names.append(info[12])

        return names

    def get_joint_names(self):
        """
        Returns a list of all joint names
        """
        names = []
        for info in self.joint_infos:
            names.append(info[1])

        return names

    def check_contact(self, list = []):
        """
        Checks for contacts between bodyID and given list of links indices.
        """
        if not list:
            list = range(self.num_joints)

        for i in list:
            cont = p.getContactPoints(self.arm[0], -1, i)
            if cont: return True
        return False

    # END-EFFECTOR SPECIFIC IMPLEMENTATIONS FOR CONVENIENCE
    # ----------------------------------------------------------------------------------------------------------------

    def get_manipulability_ellipsoid(self):
        """
        Get manipulability ellipsoid of the end effector
        """
        J = self.get_end_effector_jacobian()
        el = np.linalg.pinv(np.matmul(J, np.transpose(J)))
        return el

    def get_end_effector_state(self):
        """
        Get the state of the end-effector
        """
        result = self._get_link_state(self._ee_link_index)
        self.ee_link_trn, self.ee_link_rot, self.ee_com_trn, self.ee_com_rot, self.ee_frame_pos, self.ee_frame_rot = result

    def get_end_effector_pose(self):
        """
        Using pybullet frame info, grab the current global pose as a numpy array
        """
        # pose of end effector frame in the world frame
        ee_pose = self._get_link_pose(self._ee_link_index)
        return ee_pose

    def get_end_effector_jacobian(self):
        """
        Grab the rotational and translational jacobian of the end effector
        """
        zero_vec = [0.0] * len(self.jnt_pos)
        self.get_end_effector_state()
        jac_t, jac_r = p.calculateJacobian(self.arm[0], self._ee_link_index, self.ee_com_trn, self.jnt_pos, zero_vec,
                                           zero_vec)
        J = np.concatenate((jac_t, jac_r), axis=0)
        return J

    # SET GOALS
    # ----------------------------------------------------------------------------------------------------------------

    def set_control_method(self, m):
        """
        Sets the control method variable
        """
        self._control_method = m

    def set_joint_position_goal(self, cmd):
        """
        Set goal joint position
        """
        self.pos_cmd = cmd

    def set_joint_velocity_goal(self, cmd):
        """
        Set goal joint velocity
        """
        self.vel_cmd = cmd

    def set_frame_pose_goal(self, index, t_pos, t_rot):
        ''' set a pose goal for an arbitrary frame'''
        result = p.calculateInverseKinematics(self.arm[0], index, targetPosition=t_pos.tolist(),
                                              targetOrientation=t_rot.tolist(), maxNumIterations = 200, residualThreshold = 0.002)
        self.pos_cmd = result
        return self.pos_cmd

    def set_frame_position_goal(self, index, t_pos):
        ''' set a pose goal for an arbitrary frame'''
        result = p.calculateInverseKinematics(self.arm[0], index, targetPosition=t_pos.tolist(), maxNumIterations = 200, residualThreshold = 0.002)
        self.pos_cmd = result
        return self.pos_cmd

    def set_frame_velocity_goal(self, index, t_vel):
        """
        Set Cartesian velocity goal for arbitrary frame
        """
        J = self.get_end_effector_jacobian()
        result = np.dot(np.linalg.pinv(J), t_vel)
        self.vel_cmd = result

    def close_gripper(self):
        """
        Close the robot gripper (modifies the current joint position command)
        """
        _cmd = list(self.pos_cmd)
        for i in self._gripper_indices:
            _cmd[i] = 1
        self.pos_cmd = tuple(_cmd)

    def open_gripper(self):
        """
        Open the robot gripper (modifies the current joint position command)
        """
        _cmd = list(self.pos_cmd)
        for i in self._gripper_indices:
            _cmd[i] = 0
        self.pos_cmd = tuple(_cmd)

    # UPDATE INTERNALLY
    # ----------------------------------------------------------------------------------------------------------------
    def update(self):
        """
        This function should be configurable
        """
        # run iteration of control loop
        if self._control_method == 'p':
            self._joint_position_control(self.pos_cmd)
            #self._hard_set_joint_positions(self.pos_cmd)
        elif self._control_method == 'v':
            self._joint_velocity_control(self.vel_cmd)

        # get joint positions, velocities, torques
        self._get_joint_states()
