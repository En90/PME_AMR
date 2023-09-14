#!/home/en/python3.8_venv/bin/python3

#from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
        packages=['firebase_bridge'],
        package_dir={'': 'src'}
        )

setup(**setup_args)
