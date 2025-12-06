import os
import math
import numpy as np
import gym
from gym import spaces
import pybullet as p
import pybullet_data


class NAOEnv(gym.Env):
    """Gym environment wrapping the NAO URDF in PyBullet.

    Observation: [joint_positions, joint_velocities, base_roll, base_pitch, base_z, base_lin_vel_x]
    Action: desired position offsets for the controlled joints in radians (clipped)
    """

    def __init__(self, urdf_path=None, gui=False, time_step=1.0/240.0, frame_skip=8):
        super().__init__()
        self.gui = gui
        self.time_step = time_step
        self.frame_skip = frame_skip

        # find URDF
        pkg_dir = os.path.dirname(os.path.abspath("__file__"))

        default_urdf = os.path.join(pkg_dir, 'nao_desc', 'nao.urdf')
        self.urdf = urdf_path or default_urdf

        # connect
        if self.gui:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)

        # placeholder
        self.robot = None

        # controlled joints (pitch joints for walking)
        self.ctrl_joint_names = ['LHipPitch', 'LKneePitch', 'LAnklePitch',
                                 'RHipPitch', 'RKneePitch', 'RAnklePitch']

        # action / observation spaces will be set after loading
        self.action_space = None
        self.observation_space = None

        self.reset()

    def _load(self):
        # load plane and robot
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.loadURDF('plane.urdf')
        self.robot = p.loadURDF(self.urdf, [0, 0, 0.42], useFixedBase=False)

        # map joints
        self.joints = {}
        for i in range(p.getNumJoints(self.robot)):
            info = p.getJointInfo(self.robot, i)
            name = info[1].decode('utf-8')
            self.joints[name] = i

        self.ctrl_joints = [self.joints[n] for n in self.ctrl_joint_names if n in self.joints]

        # observation: positions + velocities + base roll/pitch + base z + base vx
        obs_dim = len(self.ctrl_joints) * 2 + 4
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        # action: desired joint position offsets (radians)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(len(self.ctrl_joints),), dtype=np.float32)

        # increase foot friction
        for name, idx in self.joints.items():
            if name in ('l_sole', 'r_sole', 'l_foot', 'r_foot'):
                try:
                    p.changeDynamics(self.robot, idx, lateralFriction=1.0)
                except Exception:
                    pass

    def reset(self):
        if self.robot is None:
            self._load()
        else:
            self._load()

        # small randomization
        for j in self.ctrl_joints:
            p.resetJointState(self.robot, j, targetValue=0.0, targetVelocity=0.0)

        self.prev_base_pos, _ = p.getBasePositionAndOrientation(self.robot)
        obs = self._get_obs()
        return obs

    def _get_obs(self):
        q = []
        dq = []
        for j in self.ctrl_joints:
            st = p.getJointState(self.robot, j)
            q.append(st[0])
            dq.append(st[1])

        base_pos, base_ori = p.getBasePositionAndOrientation(self.robot)
        base_vel_lin, base_vel_ang = p.getBaseVelocity(self.robot)
        # roll, pitch from quaternion
        euler = p.getEulerFromQuaternion(base_ori)
        roll, pitch, _ = euler

        obs = np.concatenate([np.array(q), np.array(dq), np.array([roll, pitch, base_pos[2], base_vel_lin[0]])]).astype(np.float32)
        return obs

    def step(self, action):
        # convert action (-1..1) to position offsets (radians)
        max_offset = 0.6
        offsets = np.clip(action, -1.0, 1.0) * max_offset

        # read current positions
        current = [p.getJointState(self.robot, j)[0] for j in self.ctrl_joints]
        targets = [float(c + o) for c, o in zip(current, offsets)]

        # apply position control for frame_skip*substeps
        for _ in range(self.frame_skip):
            for j, targ in zip(self.ctrl_joints, targets):
                p.setJointMotorControl2(self.robot, j, p.POSITION_CONTROL, targetPosition=targ, force=50)
            p.stepSimulation()

        obs = self._get_obs()

        # reward: forward progress in base x
        base_pos, _ = p.getBasePositionAndOrientation(self.robot)
        vx = (base_pos[0] - self.prev_base_pos[0]) / (self.time_step * self.frame_skip)
        self.prev_base_pos = base_pos

        reward = vx * 10.0 - 0.01 * float(np.sum(np.abs(action)))

        # terminate if robot falls (base too low or too tilted)
        done = False
        if base_pos[2] < 0.20:
            done = True
            reward -= 5.0

        # observation also contains roll/pitch; penalize large tilts
        roll = obs[-4]
        pitch = obs[-3]
        if abs(roll) > 0.8 or abs(pitch) > 1.0:
            done = True
            reward -= 5.0

        return obs, float(reward), bool(done), {}

    def render(self, mode='human'):
        if self.gui:
            # PyBullet GUI is already showing
            time.sleep(self.time_step)

    def close(self):
        try:
            p.disconnect()
        except Exception:
            pass
