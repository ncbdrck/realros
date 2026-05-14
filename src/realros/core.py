#!/bin/python3
"""
Thin re-export of the canonical gym proxy class.

The actual implementation lives in ``uniros._proxy.GymProxy``. Before
Round 8 the same class was triplicated here, in multiros/core.py, and
in uniros/core.py; every bug fix had to land three times. Now it is
defined once in UniROS and re-exported by multiros and realros.

Usage (unchanged from earlier versions):
    from realros.core import RealrosGym as gym
    env = gym.make("env_name", args)
    env.reset()
"""

from uniros._proxy import GymProxy

# Historical class name preserved for backwards compatibility.
# ``RealrosGym`` is an alias for ``GymProxy``; ``isinstance(env,
# RealrosGym)`` continues to return True for envs created through
# any of the three packages.
RealrosGym = GymProxy

__all__ = ["RealrosGym", "GymProxy"]
