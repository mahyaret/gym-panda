from setuptools import setup
from pathlib import Path

setup(name='gym_panda',
      version='0.0.1',
      long_description=Path("README.md").read_text(),
      install_requires=['gym']  # And any other dependencies foo needs
)
