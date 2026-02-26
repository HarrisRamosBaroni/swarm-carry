# mujoco_multi_agv/mujoco_interface.py

import mujoco
import numpy as np


class MujocoInterface:

    def __init__(self, model_path, num_robots):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.num_robots = num_robots
        self.actuator_ids = self._get_actuator_ids()
        self.body_ids = self._get_body_ids()

    def _get_actuator_ids(self):
        actuators = {}
        for i in range(self.num_robots):
            left = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                f"robot_{i}_left_actuator"
            )
            right = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                f"robot_{i}_right_actuator"
            )
            actuators[i] = (left, right)
        return actuators

    def _get_body_ids(self):
        bodies = {}
        for i in range(self.num_robots):
            body = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY,
                f"robot_{i}_base"
            )
            bodies[i] = body
        return bodies

    def set_wheel_velocity(self, robot_id, left, right):
        left_id, right_id = self.actuator_ids[robot_id]
        self.data.ctrl[left_id] = left
        self.data.ctrl[right_id] = right

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def get_robot_pose(self, robot_id):
        body_id = self.body_ids[robot_id]
        pos = self.data.xpos[body_id].copy()
        quat = self.data.xquat[body_id].copy()
        return pos, quat

    def get_robot_twist(self, robot_id):
        body_id = self.body_ids[robot_id]
        vel = self.data.cvel[body_id].copy()
        return vel