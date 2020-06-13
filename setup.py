import setuptools
from pathlib import Path

setuptools.setup(
    name='gym_panda',
    author="Mahyar Abdeetedal",
    author_email="mahyar.etedal@icloud.com",
    version='0.0.6',
    description="An OpenAI Gym Env for Panda",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    url="https://github.com/mahyaret/gym-panda",
    packages=setuptools.find_packages(include="gym_panda*"),
    install_requires=['gym', 'pybullet', 'numpy'],  # And any other dependencies foo needs
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        ],
    python_requires='>=3.6'
)
