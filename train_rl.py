import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
import argparse
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv




from envs.nao_env import NAOEnv


def make_env(gui=False):
    def _init():
        return NAOEnv(gui=gui)
    return _init


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=200_000)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--save', default='ppo_nao')
    args = parser.parse_args()

    env = DummyVecEnv([make_env(gui=args.gui)])

    model = PPO('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=args.timesteps)
    model.save(args.save)

    print(f"Model saved to {args.save}")


if __name__ == '__main__':
    main()
