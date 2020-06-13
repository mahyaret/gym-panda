# Basic package test
import os

import gym
import gym_panda

def main():
    """
    Tests importing of gym envs
    """
    spec = gym.spec('panda-v0')

    print("Test complete.")
    return True


def test_import():
    assert main() is True


if __name__ == '__main__':
    main()
