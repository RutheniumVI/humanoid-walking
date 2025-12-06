import os
import time
import math
import argparse

import pybullet as p
import pybullet_data


def list_joints(robot):
    joints = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        joints[info[1].decode('utf-8')] = i
    return joints


def run_demo(urdf_path, gui=True, freq=1.0, duration=10.0):
    if gui:
        physics = p.connect(p.GUI)
    else:
        physics = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)

    plane = p.loadURDF("plane.urdf")

    cwd = os.path.dirname(os.path.abspath(__file__))
    urdf_full = urdf_path if os.path.isabs(urdf_path) else os.path.join(cwd, urdf_path)
    # print(f"Loading URDF from: {urdf_full}")
    robot = p.loadURDF(urdf_full, [0, 0, 0.4], useFixedBase=False)

    joints = list_joints(robot)
    print("Found joints:")
    for name, idx in joints.items():
        print(f"  {idx}: {name}")

    # Common NAO leg joint names in this URDF
    L_HIP_PITCH = joints.get('LHipPitch')
    L_HIP_ROLL = joints.get('LHipRoll')
    L_KNEE = joints.get('LKneePitch')
    L_ANKLE = joints.get('LAnklePitch')

    R_HIP_PITCH = joints.get('RHipPitch')
    R_HIP_ROLL = joints.get('RHipRoll')
    R_KNEE = joints.get('RKneePitch')
    R_ANKLE = joints.get('RAnklePitch')

    # Simple sinusoidal walking pattern parameters (conservative defaults)
    hip_amp = 0.15  # forward/back hip pitch amplitude (radians)
    hip_roll_amp = 0.12  # lateral COM shift amplitude (radians)
    knee_amp = 0.4
    ankle_amp = 0.12
    knee_offset = 0.6  # a standing knee bend
    # additional knee flex during swing to provide foot clearance
    swing_knee_extra = 0.35
    # position control gains for softer, more compliant behavior
    pos_gain = 0.8
    vel_gain = 1.0
    
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "nao_walk_demo.mp4")

    start_time = time.time()
    t = 0.0

    # increase friction on soles (helps prevent slipping)
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        link_name = info[12].decode('utf-8')
        if link_name in ('l_sole', 'r_sole', 'l_foot', 'r_foot'):
            p.changeDynamics(robot, i, lateralFriction=1.0)

    while time.time() - start_time < duration:
        t = time.time() - start_time
        phase = 2.0 * math.pi * freq * t

        left_phase = math.sin(phase)
        right_phase = math.sin(phase + math.pi)
        # simple lateral COM shift: shift toward stance foot using hip roll
        # when left swing (left_phase > 0) the right foot is stance, so roll to the right (negative left_roll)
        left_swing = left_phase > 0
        right_swing = right_phase > 0

        # hip pitch (forward/back)
        if L_HIP_PITCH is not None:
            hip_pos = hip_amp * left_phase
            p.setJointMotorControl2(robot, L_HIP_PITCH, p.POSITION_CONTROL, targetPosition=hip_pos, force=30, positionGain=pos_gain, velocityGain=vel_gain)
        if R_HIP_PITCH is not None:
            hip_pos = hip_amp * right_phase
            p.setJointMotorControl2(robot, R_HIP_PITCH, p.POSITION_CONTROL, targetPosition=hip_pos, force=30, positionGain=pos_gain, velocityGain=vel_gain)

        # hip roll for balance (COM over stance foot)
        if L_HIP_ROLL is not None:
            # when left leg swings, roll to right (negative roll), and vice versa
            roll_target = -hip_roll_amp if left_swing else hip_roll_amp * 0.2
            p.setJointMotorControl2(robot, L_HIP_ROLL, p.POSITION_CONTROL, targetPosition=roll_target, force=20, positionGain=pos_gain, velocityGain=vel_gain)
        if R_HIP_ROLL is not None:
            roll_target = hip_roll_amp if left_swing else -hip_roll_amp * 0.2
            p.setJointMotorControl2(robot, R_HIP_ROLL, p.POSITION_CONTROL, targetPosition=roll_target, force=20, positionGain=pos_gain, velocityGain=vel_gain)

        # knee (add extra flex during swing for clearance)
        if L_KNEE is not None:
            extra = swing_knee_extra if left_swing else 0.0
            knee_pos = knee_offset - (knee_amp + extra) * left_phase
            p.setJointMotorControl2(robot, L_KNEE, p.POSITION_CONTROL, targetPosition=knee_pos, force=20, positionGain=pos_gain, velocityGain=vel_gain)
        if R_KNEE is not None:
            extra = swing_knee_extra if right_swing else 0.0
            knee_pos = knee_offset - (knee_amp + extra) * right_phase
            p.setJointMotorControl2(robot, R_KNEE, p.POSITION_CONTROL, targetPosition=knee_pos, force=20, positionGain=pos_gain, velocityGain=vel_gain)
        # ankle adjusts to help foot clearance and land softly
        if L_ANKLE is not None:
            ank_pos = ankle_amp * left_phase + (0.2 * extra if left_swing else 0.0)
            p.setJointMotorControl2(robot, L_ANKLE, p.POSITION_CONTROL, targetPosition=ank_pos, force=20, positionGain=pos_gain, velocityGain=vel_gain)
        if R_ANKLE is not None:
            ank_pos = ankle_amp * right_phase + (0.2 * extra if right_swing else 0.0)
            p.setJointMotorControl2(robot, R_ANKLE, p.POSITION_CONTROL, targetPosition=ank_pos, force=20, positionGain=pos_gain, velocityGain=vel_gain)

        p.stepSimulation()
        if gui:
            time.sleep(1.0 / 240.0)

    p.disconnect()


def main():
    parser = argparse.ArgumentParser(description='Simple NAO walking demo in PyBullet')
    parser.add_argument('--urdf', default='nao_desc/nao.urdf', help='Path to NAO URDF file (relative to script)')
    parser.add_argument('--nogui', action='store_true', help='Run in DIRECT mode (no GUI)')
    parser.add_argument('--freq', type=float, default=1.0, help='Step frequency (Hz)')
    parser.add_argument('--duration', type=float, default=10.0, help='Duration (s)')
    parser.add_argument('--list-joints', action='store_true', help='List joint names and exit')

    args = parser.parse_args()

    if args.list_joints:
        # quick load just to list
        if args.nogui:
            physics = p.connect(p.DIRECT)
        else:
            physics = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        robot = p.loadURDF(args.urdf, [0, 0, 0.4])
        joints = list_joints(robot)
        for name, idx in joints.items():
            print(f"{idx}: {name}")
        p.disconnect()
        return

    run_demo(args.urdf, gui=not args.nogui, freq=args.freq, duration=args.duration)


if __name__ == '__main__':
    main()
