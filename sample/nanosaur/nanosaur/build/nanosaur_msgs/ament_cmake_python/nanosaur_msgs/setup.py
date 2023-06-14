from setuptools import find_packages
from setuptools import setup

setup(
    name='nanosaur_msgs',
    version='2.1.0',
    packages=find_packages(
        include=('nanosaur_msgs', 'nanosaur_msgs.*')),
)
