#!/home/en/python3.8_venv/bin/python3

# from distutils.core import setup
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["firebase_bridge", "firebase_bridge_class", "firebase_bridge_service"],
    package_dir={
        "": "src",
        "firebase_bridge_class": "src/firebase_bridge/firebase_bridge_class",
        "firebase_bridge_service": "src/firebase_bridge/firebase_bridge_service",
    },
    install_requires=[
        "firebase-admin",
        "pyyaml",
        "rospkg",
    ],
)

setup(**setup_args)
