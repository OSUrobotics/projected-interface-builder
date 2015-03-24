#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['projected_interface_builder'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'dynamic_reconfigure'],
    scripts=['interface_builder.py'],
)

setup(**package_info)
