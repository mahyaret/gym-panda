# gym-panda

[![Build Status](https://travis-ci.org/mahyaret/gym-panda.svg?branch=master)](https://travis-ci.org/mahyaret/gym-panda)
[![Downloads](https://pepy.tech/badge/gym-panda)](https://pepy.tech/project/gym-panda)
[![PyPI version](https://badge.fury.io/py/gym-panda.svg)](https://badge.fury.io/py/gym-panda)


OpenaAI Gym Franka Emika Panda robot grasping environment implemented with PyBullet


![](https://www.etedal.net/img/twenty/panda.gif)

## Links

- GitHub: [https://github.com/mahyaret/gym-panda](https://github.com/mahyaret/gym-panda)
- PyPI: [https://pypi.org/project/gym-panda/](https://pypi.org/project/gym-panda/)
- Documentation: [https://www.etedal.net/2020/04/pybullet-panda_2.html](https://www.etedal.net/2020/04/pybullet-panda_2.html)
- Issue Tracker:[https://github.com/mahyaret/gym-panda/issues](https://github.com/mahyaret/gym-panda/issues)
- Download: [https://pypi.org/project/gym-panda/#files](https://pypi.org/project/gym-panda/#files)


## Install

Install with `pip`:

    pip install gym-panda
    
Or, install from source:

    git clone https://github.com/mahyaret/gym-panda.git
    cd gym-panda
    pip install .

## Basic Usage

Running an environment:

```python
import gym
import gym_panda
env = gym.make('panda-v0')
env.reset()
for _ in range(100):
    env.render()
    obs, reward, done, info = env.step(
        env.action_space.sample()) # take a random action
env.close()
 ```
 
 Running a PD control HACK!
 
 ```python
import gym
import gym_panda

env = gym.make('panda-v0')
observation = env.reset()
done = False
error = 0.01
fingers = 1
object_position = [0.7, 0, 0.1]

k_p = 10
k_d = 1
dt = 1./240. # the default timestep in pybullet is 240 Hz  
t = 0

for i_episode in range(20):
    observation = env.reset()
    fingers = 1
    for t in range(100):
        env.render()
        print(observation)
        dx = object_position[0]-observation[0]
        dy = object_position[1]-observation[1]
        target_z = object_position[2] 
        if abs(dx) < error and abs(dy) < error and abs(dz) < error:
            fingers = 0
        if (observation[3]+observation[4])<error+0.02 and fingers==0:
            target_z = 0.5
        dz = target_z-observation[2]
        pd_x = k_p*dx + k_d*dx/dt
        pd_y = k_p*dy + k_d*dy/dt
        pd_z = k_p*dz + k_d*dz/dt
        action = [pd_x,pd_y,pd_z,fingers]
        observation, reward, done, info = env.step(action)
        object_position = info['object_position']
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
env.close()
 ```

## Development

- clone the repo:
```bash
git clone https://github.com/mahyaret/gym-panda.git
cd gym-panda
```
    
- Create/activate the virtual environment:
```bash
pipenv shell --python=python3.6
```

- Install development dependencies:
```bash
pipenv install --dev
```
