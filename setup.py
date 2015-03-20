## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['reconfigurable_transform_publisher', 'reconfigurable_transform_publisher.cfg'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg', 'dynamic_reconfigure', 'tf'],
    scripts=['nodes/transform_publisher.py']
)

setup(**setup_args)