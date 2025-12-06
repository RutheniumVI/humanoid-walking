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
