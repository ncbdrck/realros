from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="ros_rl",
    packages=['ros_rl'],
    package_dir={'': 'src'},

    description="A Comprehensive Framework for Real-World Robotic Reinforcement Learning",
    url="https://github.com/ncbdrck/ros_rl/",
    keywords=['ROS', 'reinforcement learning', 'machine-learning', 'gym', 'robotics', 'openai'],

    author='Jayasekara Kapukotuwa',
    author_email='j.kapukotuwa@research.ait.ie',

    license="MIT",
)

setup(**setup_args)