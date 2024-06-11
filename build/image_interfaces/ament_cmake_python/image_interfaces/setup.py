from setuptools import find_packages
from setuptools import setup

setup(
    name='image_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('image_interfaces', 'image_interfaces.*')),
)
