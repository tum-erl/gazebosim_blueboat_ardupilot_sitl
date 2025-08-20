from setuptools import find_packages
from setuptools import setup

setup(
    name='blueboat_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('blueboat_interfaces', 'blueboat_interfaces.*')),
)
