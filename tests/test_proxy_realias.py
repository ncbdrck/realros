"""
Regression: realros.core.RealrosGym must be the same class object
as uniros._proxy.GymProxy, so any fix landed in UniROS also lands
in realros.
"""
from realros.core import RealrosGym
from uniros._proxy import GymProxy
from uniros.core import uniros_gym


def test_realros_gym_is_canonical_gym_proxy():
    assert RealrosGym is GymProxy


def test_realros_gym_is_uniros_gym():
    assert RealrosGym is uniros_gym


def test_isinstance_via_realros_alias():
    assert issubclass(RealrosGym, GymProxy)
    assert issubclass(GymProxy, RealrosGym)


def test_make_classmethod_present():
    assert hasattr(RealrosGym, "make")
    assert callable(RealrosGym.make)
