from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="realros",
    packages=['realros'],
    package_dir={'': 'src'},
    # PEP 561: ship the py.typed marker so type checkers (mypy /
    # pyright) treat installed copies of the package as typed.
    package_data={'realros': ['py.typed']},

    description="A Comprehensive Framework for Real-World Robotic Reinforcement Learning",
    url="https://github.com/ncbdrck/realros/",
    keywords=['ROS', 'reinforcement learning', 'machine-learning', 'gym', 'robotics', 'openai'],

    author='Jayasekara Kapukotuwa',
    author_email='j.kapukotuwa@research.ait.ie',

    license="MIT",
)

setup(**setup_args)