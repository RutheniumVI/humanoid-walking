import argparse
import time
import os
import numpy as np
from stable_baselines3 import PPO

from envs.nao_env import NAOEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='ppo_nao.zip')
    parser.add_argument('--duration', type=float, default=20.0)
    args = parser.parse_args()

    env = NAOEnv(gui=True)
    model = PPO.load(args.model, env=env)

    obs = env.reset()
    start = time.time()
    while time.time() - start < args.duration:
        action, _ = model.predict(obs, deterministic=True)
        obs, rew, done, _ = env.step(action)
        if done:
            obs = env.reset()

    env.close()


if __name__ == '__main__':
    main()
