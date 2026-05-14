"""
Regression: realros's kill_all_ros_processes + kill_all_roslaunch_process
were renamed to kill_all_HOST_ros_processes / kill_all_HOST_roslaunch_processes
to make their host-wide ``killall -9`` scope explicit. The old names
remain as DeprecationWarning-emitting aliases.
"""
import warnings

import pytest

from realros.utils import ros_common


class TestNewNamesExist:
    def test_kill_all_host_ros_processes_callable(self):
        assert callable(ros_common.kill_all_host_ros_processes)

    def test_kill_all_host_roslaunch_processes_callable(self):
        assert callable(ros_common.kill_all_host_roslaunch_processes)


class TestOldNamesStillWork:
    def test_old_kill_all_ros_processes_emits_warning(self, monkeypatch):
        called = {"flag": False}
        def _fake_impl():
            called["flag"] = True
            return True
        monkeypatch.setattr(
            ros_common, "kill_all_host_ros_processes", _fake_impl
        )
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.kill_all_ros_processes()
        assert called["flag"]
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_old_kill_all_roslaunch_process_emits_warning(self, monkeypatch):
        called = {"flag": False}
        def _fake_impl():
            called["flag"] = True
            return True
        monkeypatch.setattr(
            ros_common, "kill_all_host_roslaunch_processes", _fake_impl
        )
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.kill_all_roslaunch_process()
        assert called["flag"]
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)
