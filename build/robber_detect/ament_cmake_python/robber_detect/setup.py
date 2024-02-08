from setuptools import find_packages
from setuptools import setup

setup(
    name='robber_detect',
    version='0.0.0',
    packages=find_packages(
        include=('robber_detect', 'robber_detect.*')),
)
