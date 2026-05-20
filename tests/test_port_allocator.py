"""
Regression: launch_roscore uses socket.bind(0) to pick a free port
atomically. realros has a single-port API (returns just ``ros_port``
as a string) and an extra ``default_port`` argument; otherwise the
allocator is the same as multiros.
"""
import socket
import threading
import warnings

import pytest

from realros.utils import ros_common


class _NoopPopen:
    instances = []
    def __init__(self, cmd, *a, **k):
        self.cmd = cmd
        type(self).instances.append(self)
        self._alive = True
    def poll(self):
        return None if self._alive else 0
    def terminate(self):
        self._alive = False
    def kill(self):
        self._alive = False
    def wait(self):
        return 0


@pytest.fixture
def patched_launch(monkeypatch, tmp_path):
    import subprocess as sp
    import time as _time
    _NoopPopen.instances.clear()
    monkeypatch.setattr(sp, "Popen", _NoopPopen)
    monkeypatch.setattr(_time, "sleep", lambda *a, **k: None)
    monkeypatch.setattr(ros_common, "change_ros_master", lambda *a, **k: True)
    monkeypatch.setattr(ros_common, "_PORT_LOG_PATH", str(tmp_path / "ports.log"))
    # launch_roscore now verifies the spawned roscore is reachable before
    # returning. The fixture's Popen is a no-op so no real roscore exists;
    # stub the reachability probe to True so the retry loop doesn't fire.
    monkeypatch.setattr(ros_common, "_master_is_reachable", lambda *a, **k: True)
    yield


class TestKernelAllocator:
    def test_reserve_free_port_returns_open_port(self):
        p = ros_common._reserve_free_port()
        assert 1024 <= p < 65536
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', p))
        finally:
            sock.close()

    def test_port_is_free_returns_false_for_squatted_port(self):
        squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        squat.bind(('127.0.0.1', 0))
        squat.listen(1)
        squatted = squat.getsockname()[1]
        try:
            assert ros_common._port_is_free(squatted) is False
        finally:
            squat.close()


class TestLaunchRoscore:
    def test_returns_single_port_string(self, patched_launch):
        ros_p = ros_common.launch_roscore(set_new_master_vars=False)
        assert isinstance(ros_p, str)
        assert int(ros_p) > 0

    def test_serial_calls_get_unique_ports(self, patched_launch):
        ports = [
            ros_common.launch_roscore(set_new_master_vars=False)
            for _ in range(10)
        ]
        assert len(set(ports)) == len(ports), f"Duplicates in {ports}"

    def test_parallel_calls_get_unique_ports(self, patched_launch):
        results = []
        lock = threading.Lock()
        def _launch():
            p = ros_common.launch_roscore(set_new_master_vars=False)
            with lock:
                results.append(p)
        threads = [threading.Thread(target=_launch) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert len(set(results)) == len(results)

    def test_default_port_11311_when_free(self, patched_launch):
        # If 11311 is currently free on the test host, default_port=True
        # picks it. If it's not free, the legacy fallback path runs and
        # we accept any port. The point is: doesn't crash.
        ros_p = ros_common.launch_roscore(default_port=True, set_new_master_vars=False)
        assert int(ros_p) > 0

    def test_requested_port_falls_back_when_taken(self, patched_launch):
        squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        squat.bind(('127.0.0.1', 0))
        squat.listen(1)
        squatted = squat.getsockname()[1]
        try:
            ros_p = ros_common.launch_roscore(
                port=squatted, set_new_master_vars=False
            )
            assert int(ros_p) != squatted
        finally:
            squat.close()


class TestLegacyHelperDeprecation:
    def test_get_all_the_ros_masters_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.get_all_the_ros_masters()
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_add_to_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.add_to_rosmaster_list("11500")
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_remove_from_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.remove_from_rosmaster_list("11500")
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_remove_all_from_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.remove_all_from_rosmaster_list()
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)
