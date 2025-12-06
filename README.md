# humanoid-walking
This repository contains a NAO robot URDF and PyBullet demos, plus a simple RL setup to train a walking policy.

Quick start

1. Create a Python environment and install dependencies (PyTorch may require a platform-specific install):

```powershell
python -m pip install -r requirements.txt
```

2. Run the heuristic demo (visual):

```powershell
python walk_nao.py --duration 20
```

3. Train a PPO policy (this will be slow without GPU):

```powershell
python train_rl.py --timesteps 200000
```

4. Visualize a trained policy in GUI:

```powershell
python run_policy.py --model ppo_nao.zip --duration 40
```

5. Print joint names for debugging:

```powershell
python walk_nao.py --list-joints
```

Notes

- `train_rl.py` uses `stable-baselines3` (PPO) and the `envs.nao_env.NAOEnv` Gym wrapper. The environment observation is a compact vector of joint angles, velocities and base state. The action is desired joint position offsets for the leg joints.
- The included environment and reward are intentionally simple to get you started. Training robust locomotion requires reward shaping, additional observation (contacts, IMU), curriculum, and longer training times.
- On Windows install PyTorch according to the official instructions (`https://pytorch.org/get-started/locally/`) before installing `stable-baselines3` if you want GPU acceleration.

If you'd like, I can:
- Add contact-based observations and better reward shaping (distance to target, energy penalties).
- Provide a tested training config and a smaller smoke-test training schedule.
- Instrument training with TensorBoard logging and checkpointing.

Enjoy experimenting!
