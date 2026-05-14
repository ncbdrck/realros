#!/bin/python3
"""
Re-export of the canonical multiprocessing gym proxy.

The implementation lives in :class:`uniros._proxy.GymProxy`.
``RealrosGym`` is an alias kept for backwards compatibility — both
names refer to the same class object, so ``isinstance(env,
RealrosGym)`` matches envs created through any package in the
ecosystem.

Usage::

    from realros.core import RealrosGym as gym
    env = gym.make("env_name", args)
    env.reset()
"""

from uniros._proxy import GymProxy

RealrosGym = GymProxy

__all__ = ["RealrosGym", "GymProxy"]
