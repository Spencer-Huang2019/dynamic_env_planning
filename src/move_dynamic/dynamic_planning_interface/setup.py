from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d["packages"] = ["move_dynamic_planning_interface"]
d["package_dir"] = {"": "python"}

setup(**d)