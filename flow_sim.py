"""
Created by Ale and Nati
"""
from collections import OrderedDict
from time import time

import numpy as np


class Natgorithm:

    def __init__(self):
        self.actuator_positions = OrderedDict({1: [10.0, 10.0],
                                               2: [3.0, 17.0]})
        self.actuator_size = 20
        self.vortex_positions = []
        self.t_prev = time()
        self.t_last_vortex = time()
        self.p_prev = np.array((0.0, 0.0, 0.0))

    @staticmethod
    def v_global_to_v_hand(v):
        return np.array((v[0], v[1]))

    def actuators_ordered_along_vect(self, v):
        angle = np.pi * 0.5 - np.arctan2(v[1], v[0])
        rot_mat = np.array([[np.cos(angle), -np.sin(angle)],
                            [np.sin(angle), np.cos(angle)]])
        rotated_sensors = {}
        for sensor_num, sensor_vect in self.actuator_positions.items():
            rotated_sensors[sensor_num] = np.dot(rot_mat, sensor_vect)

        return OrderedDict(reversed(sorted(rotated_sensors.items(), key=lambda x: x[1][1])))

    @staticmethod
    def distances_along_v(positions_list):
        positions_list = np.array(positions_list)
        distances = positions_list[1:] - positions_list[:-1]
        return distances

    def run(self):
        t_now = time()
        dt = t_now - self.t_prev
        position = np.array((4.9, 3.0, 5.1))
        v_global = (position - self.p_prev) / dt
        # v_global = np.array((350.0, 150.0))  # Used for testing
        v_hand = self.v_global_to_v_hand(v_global)
        ordered_actuators = self.actuators_ordered_along_vect(v_hand)

        t_temp = time()
        if 0.45 / np.clip(np.linalg.norm(v_hand), 0.001, 10.0) <= (t_temp - self.t_last_vortex):
            self.vortex_positions.append(np.array((0.0, 200.0)))
            self.t_last_vortex = t_temp

        self.vortex_positions = [np.array((0.0, vortex_position[1] - np.linalg.norm(v_hand) * dt)) for vortex_position
                                 in self.vortex_positions if
                                 vortex_position[1] >= 0]

        actuators_to_actuate = []
        for actuator_id, actuator_pos in ordered_actuators.items():
            for vortex_position in self.vortex_positions:
                if np.linalg.norm(vortex_position - actuator_pos) <= self.actuator_size:
                    actuators_to_actuate.append(actuator_id)
                    break

        self.p_prev = position
        self.t_prev = t_now

        return actuators_to_actuate


if __name__ == "_main_":
    vel = [10.0, -15.0, 20.0]
    test_inst = Natgorithm()
    print(test_inst.v_global_to_v_hand(vel))
    v_list_test = test_inst.actuators_ordered_along_vect(vel)
    print(v_list_test)
    print(test_inst.distances_along_v(list(v_list_test.values())))