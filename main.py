import pybullet as p
import pybullet_data
import time
import math

class HumanoidSimulator:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = self.load_humanoid()
        self.joint_names, self.joint_indices = self.get_joint_info()
        self.leg_joints = self.find_leg_joints()
        self.controlled_joints = [self.joint_indices[name] for name in self.leg_joints]
        self.set_position_control_mode()

    def load_humanoid(self):
        robots = p.loadMJCF(pybullet_data.getDataPath() + "/mjcf/humanoid.xml")
        print("[humanoid] robots:", robots)
        if not robots:
            raise RuntimeError("[humanoid] Failed to load MJCF model!")
        # Use the first robot in the list
        robot_id = robots[1]
        print("[humanoid] Humanoid robot loaded with ID:", robot_id)
        # Let everything settle
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1. / 240.)
        return robot_id

    def get_joint_info(self):
        num_joints = p.getNumJoints(self.robot_id)
        joint_names = [p.getJointInfo(self.robot_id, i)[1].decode('utf-8') for i in range(num_joints)]
        joint_indices = {name: i for i, name in enumerate(joint_names)}
        print("[humanoid] Available joint names:")
        for i, name in enumerate(joint_names):
            print(f"{i}: {name}")
        return joint_names, joint_indices

    def find_leg_joints(self):
        # Update these names based on the printed joint names if needed
        candidate_joints = [
            "left_hip_x", "left_knee", "right_hip_x", "right_knee"
        ]
        # Filter only those that exist in the model
        found = [name for name in candidate_joints if name in self.joint_indices]
        if len(found) < len(candidate_joints):
            print("[humanoid] Warning: Some leg joints not found in model:", set(candidate_joints) - set(found))
        return found

    def set_position_control_mode(self):
        for j in self.controlled_joints:
            p.setJointMotorControl2(self.robot_id, j, controlMode=p.POSITION_CONTROL, force=100)

    def run(self):
        t = 0
        while True:
            t += 0.01
            # Simple walking pattern
            left_hip_angle  = 0.3 * math.sin(2 * math.pi * t)
            left_knee_angle = 0.6 * math.sin(2 * math.pi * t)
            right_hip_angle = 0.3 * math.sin(2 * math.pi * t + math.pi)
            right_knee_angle = 0.6 * math.sin(2 * math.pi * t + math.pi)
            target_positions = [left_hip_angle, left_knee_angle, right_hip_angle, right_knee_angle]
            for idx, target in zip(self.controlled_joints, target_positions):
                p.setJointMotorControl2(self.robot_id, idx, controlMode=p.POSITION_CONTROL,
                                        targetPosition=target, force=100)
            p.stepSimulation()
            time.sleep(1. / 240.)

if __name__ == "__main__":
    sim = HumanoidSimulator()
    sim.run()