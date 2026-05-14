#! /usr/bin/env python

"""
This script is to specify all the common functions related to the handling of ROS.
It has the following functions,
    01. launch_roscore: Launch a rocore with a given or random (no overlapping) port
    02. get_all_the_ros_masters: Get all the current ros master ports
    03. add_to_rosmaster_list: Add a port to the current ros master ports list
    04. remove_from_rosmaster_list: remove a port from the current ros master ports list
    05. kill_all_host_ros_processes: Kill ALL ROS processes on this host.
    06. kill_all_host_roslaunch_processes: Kill ALL roslaunch processes on this host.
    07. kill_all_ros_nodes: Kill all running ROS nodes of a specified or the current ROS master.
    08. kill_ros_node: Kill a single ROS node of a given or current ROS master.
    09. ros_kill_master: Kill a ROS master
    10. clean_ros_logs: Clean all ros logs
    11. source_workspace: Source a ros workspace
    12. ros_launch_launcher: Execute a roslaunch with args
    13. ros_node_launcher: Launch a ROS node from ROS package
    14. ros_load_yaml: Fetch a YAML file from a package or a path and load it into the ROS Parameter Server.
    15. load_urdf: Load a URDF into the parameter server or a string containing the processed URDF data
    16. is_roscore_running: Check if a roscore is currently running
    17. change_ros_master: Set the current ROS Master Environment Variable.
    18. change_ros_master_multi_device: Set the current ROS Master Environment Variable for multi-device
    19. init_robot_state_publisher: Initialize the robot state publisher.
    20. remove_all_from_rosmaster_list: Remove all ports from the Multiros rosmaster port list.

"""
import rosparam
import rospy
import rospkg
import os
import subprocess
import time
import random
import socket
import warnings
import fcntl
import atexit
import signal
import threading
import xacro
from typing import Tuple, Union, List, Dict, Any

# Best-effort log of ports we've launched roscores on. Not load-bearing
# for allocation correctness: the kernel decides what's free via
# socket.bind(0); this file is just for diagnostics. flock-protected so
# parallel multiros/realros scripts can both append safely. The path is
# shared with multiros intentionally.
_PORT_LOG_PATH = '/tmp/ros_master_ports_multiros.txt'

# ----------------------------------------------------------------------
# Managed-process registry: track roscore processes this Python process
# spawned so we can clean them up on Ctrl+C or normal exit. Scoped to
# processes WE launched — pre-existing ROS sessions on the host are NOT
# affected (unlike kill_all_host_ros_processes).
# ----------------------------------------------------------------------
_managed_lock = threading.Lock()
_managed_processes: List[Dict[str, Any]] = []
_cleanup_done = False
_handlers_installed = False
_prev_sigint_handler: Any = None


def register_managed_process(popen, **selectors) -> None:
    """
    Register a process spawned by this script for automatic cleanup
    on Ctrl+C or normal interpreter exit.

    Args:
        popen: A ``subprocess.Popen`` (typically the xterm wrapper
            shell). ``.terminate()`` will be called on cleanup.
        **selectors: Optional fallback identifiers for cleanup when
            killing the Popen alone is not enough. Recognised keys:
              - ``roscore_port`` (str|int): triggers
                ``pkill -f "roscore -p <port>"``.
              - ``kind`` (str): free-form label for logging.
    """
    global _handlers_installed, _prev_sigint_handler
    with _managed_lock:
        _managed_processes.append({"popen": popen, "selectors": dict(selectors)})
        already_installed = _handlers_installed
        _handlers_installed = True

    if not already_installed:
        atexit.register(_cleanup_managed_processes)
        try:
            _prev_sigint_handler = signal.signal(signal.SIGINT, _sigint_handler)
        except ValueError:
            _prev_sigint_handler = None

        # Also register with rospy's shutdown machinery. ``rospy.init_node``
        # (typically called by the user's script AFTER launch_roscore)
        # installs its OWN SIGINT handler that overwrites ours — so our
        # SIGINT handler would never fire on Ctrl+C. rospy's shutdown
        # callback list is iterated regardless of who owns the signal
        # handler, so this is what actually triggers cleanup in the
        # ``import realros; rospy.init_node; train`` flow.
        try:
            rospy.on_shutdown(_cleanup_managed_processes)
        except Exception:
            pass


def _sigint_handler(signum, frame):
    """
    On Ctrl+C: tear down managed processes once, then chain to the
    previously-installed SIGINT handler (or KeyboardInterrupt). A
    second Ctrl+C during cleanup hits the chained handler and bails
    out immediately.
    """
    try:
        signal.signal(signal.SIGINT, _prev_sigint_handler if _prev_sigint_handler else signal.SIG_DFL)
    except (ValueError, TypeError):
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        rospy.loginfo("SIGINT received; cleaning up spawned roscore...")
    except Exception:
        print("[realros] SIGINT received; cleaning up spawned roscore...")

    _cleanup_managed_processes()

    if callable(_prev_sigint_handler) and _prev_sigint_handler not in (signal.SIG_DFL, signal.SIG_IGN):
        try:
            _prev_sigint_handler(signum, frame)
            return
        except Exception:
            pass
    raise KeyboardInterrupt


def _cleanup_managed_processes() -> None:
    """Tear down every registered managed process. Idempotent."""
    global _cleanup_done
    with _managed_lock:
        if _cleanup_done:
            return
        _cleanup_done = True
        to_cleanup = list(_managed_processes)
        _managed_processes.clear()

    if not to_cleanup:
        return

    for entry in to_cleanup:
        popen = entry["popen"]
        selectors = entry["selectors"]

        try:
            if popen is not None and popen.poll() is None:
                popen.terminate()
        except Exception:
            pass

        port = selectors.get("roscore_port")
        if port:
            try:
                subprocess.run(
                    ["pkill", "-f", f"roscore -p {port}"],
                    timeout=5,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass

    time.sleep(0.5)

    for entry in to_cleanup:
        popen = entry["popen"]
        try:
            if popen is not None and popen.poll() is None:
                popen.kill()
        except Exception:
            pass

    # Restore SIGINT to the default handler. Useful when this cleanup
    # runs via rospy.on_shutdown: rospy may have installed its own
    # SIGINT handler that doesn't terminate the script, so the user
    # has to Ctrl+C multiple times. Resetting to SIG_DFL here means
    # any further Ctrl+C kills the process immediately.
    try:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    except (ValueError, OSError):
        pass


def _port_is_free(port: int) -> bool:
    """
    Return True iff ``port`` can be bound on localhost right now.
    Uses SO_REUSEADDR so a recent TIME_WAIT bind doesn't falsely
    report the port as occupied.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
        except OSError:
            return False
        return True
    finally:
        sock.close()


def _reserve_free_port() -> int:
    """
    Ask the kernel for a free ephemeral port via bind(0). Standard
    pytest-xdist/portpicker pattern; eliminates the multi-process race
    inherent in the old text-file allocator.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]
    finally:
        sock.close()


def _append_port_log(ros_port: str) -> None:
    """Append ``ros_port`` to the diagnostic port log under flock."""
    try:
        with open(_PORT_LOG_PATH, 'a') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                f.write(ros_port + ' ')
                f.flush()
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError as e:
        rospy.logwarn(f"Could not write to port log {_PORT_LOG_PATH}: {e}")


def _remove_from_port_log(ros_port: str) -> None:
    """Best-effort removal of ``ros_port`` from the diagnostic log."""
    try:
        if not os.path.exists(_PORT_LOG_PATH):
            return
        with open(_PORT_LOG_PATH, 'r+') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                contents = f.read()
                tokens = [t for t in contents.split() if t and t != ros_port]
                f.seek(0)
                f.truncate()
                if tokens:
                    f.write(' '.join(tokens) + ' ')
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError as e:
        rospy.logwarn(f"Could not edit port log {_PORT_LOG_PATH}: {e}")


def launch_roscore(port: int = None, set_new_master_vars: bool = True, default_port: bool = False) -> str:
    """
    Launch a roscore on a free port and (optionally) point this process's
    ``ROS_MASTER_URI`` at it.

    Ports are allocated via ``socket.bind(('127.0.0.1', 0))`` so the
    kernel picks free ports and serializes between parallel callers.

    Args:
        port (int): A specific desired port for ``ROS_MASTER_URI``.
            If the port is unavailable on this host right now, falls
            back to a kernel-allocated free port and logs a warning.
        set_new_master_vars (bool): change the current
            ``ROS_MASTER_URI`` environment variable to the selected port.
        default_port (bool): If True, request port 11311. If 11311 is
            already in use, ``ROS_MASTER_URI`` is still pointed at it
            so the caller can attach to an existing roscore.

    Returns:
        str: ``ROS_MASTER_URI`` port as a string.
    """
    if default_port:
        port = 11311

    if port is not None and _port_is_free(port):
        ros_port = str(port)
    elif port == 11311:
        # Legacy behaviour: if the user explicitly asked for the
        # default port and it's already taken, assume an existing
        # roscore is what they want to attach to.
        rospy.logwarn("The default port 11311 is already in use; assuming existing roscore.")
        if set_new_master_vars:
            change_ros_master(str(port))
        return str(port)
    else:
        if port is not None:
            rospy.logwarn(
                f"Requested port {port} is unavailable; falling back "
                f"to a kernel-allocated free port."
            )
        ros_port = str(_reserve_free_port())

    # Diagnostic log only; not load-bearing.
    _append_port_log(ros_port)

    # launch roscore as a term command
    term_cmd = "roscore -p " + ros_port
    term_cmd = "xterm -e ' " + term_cmd + "'"
    roscore_proc = subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)
    rospy.loginfo("Roscore launched! with port: " + ros_port)

    # Register for Ctrl+C / atexit cleanup. The xterm wrapper may
    # detach, so we also record the port so cleanup can issue a
    # targeted ``pkill -f "roscore -p <port>"`` scoped to OUR roscore.
    register_managed_process(roscore_proc, roscore_port=ros_port, kind="roscore")

    # change the current ROS_MASTER to the selected one
    if set_new_master_vars:
        change_ros_master(ros_port)

    return ros_port


def get_all_the_ros_masters() -> str:
    """
    DEPRECATED. Reads the diagnostic port log only.

    Port allocation is handled by ``launch_roscore`` via
    ``socket.bind(0)``; this helper returns whatever the best-effort
    diagnostic log contains.
    """
    warnings.warn(
        "get_all_the_ros_masters() is deprecated and now reads a "
        "best-effort diagnostic log only. Port allocation is handled "
        "directly by the kernel via socket.bind(0).",
        DeprecationWarning, stacklevel=2,
    )
    if not os.path.exists(_PORT_LOG_PATH):
        return ''
    try:
        with open(_PORT_LOG_PATH, 'r') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_SH)
                return f.read().strip()
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError:
        return ''


def add_to_rosmaster_list(ros_port: str) -> bool:
    """
    DEPRECATED. Appends to the diagnostic port log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "add_to_rosmaster_list() is deprecated and now only writes "
        "to a diagnostic log. Port allocation is handled by "
        "socket.bind(0) inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    _append_port_log(ros_port)
    return True


def remove_from_rosmaster_list(ros_port: str) -> bool:
    """
    DEPRECATED. Removes ``ros_port`` from the diagnostic log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "remove_from_rosmaster_list() is deprecated and now only edits "
        "a diagnostic log. Port allocation is handled by socket.bind(0) "
        "inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    _remove_from_port_log(ros_port)
    return True


def kill_all_host_ros_processes() -> bool:
    """
    Kill EVERY rosmaster/roslaunch/rosout/robot_state_publisher/nodelet
    process on this host (not just the ones this script spawned).

    Implemented via ``killall -9`` so it is host-scoped: if another
    user or another script on the same machine is running ROS, this
    will kill their processes too. Use with care.

    Renamed from ``kill_all_ros_processes`` to make the host-wide scope
    explicit. The old name is kept as a DeprecationWarning-emitting
    alias for backwards compatibility.

    Returns:
        bool: True if the kill command ran (does not verify each
              process actually died).
    """

    term_cmd = "killall -9 rosmaster roslaunch rosout robot_state_publisher nodelet"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    # clear the diagnostic port log
    try:
        if os.path.exists(_PORT_LOG_PATH):
            with open(_PORT_LOG_PATH, 'w') as f:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    f.write('')
                finally:
                    try:
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                    except OSError:
                        pass
    except OSError:
        pass

    rospy.logdebug("Killed all host-wide ROS processes.")

    return True


def kill_all_ros_processes() -> bool:
    """
    DEPRECATED. Use :func:`kill_all_host_ros_processes` instead.

    The old name was misleading: it sounds like it cleans up only the
    instances this script spawned, but it actually issues ``killall -9``
    against rosmaster/roslaunch/rosout/etc. host-wide.
    """
    warnings.warn(
        "kill_all_ros_processes() is deprecated; use "
        "kill_all_host_ros_processes() instead. The behaviour is "
        "unchanged but the name now makes the host-wide scope explicit.",
        DeprecationWarning, stacklevel=2,
    )
    return kill_all_host_ros_processes()


def kill_all_host_roslaunch_processes() -> bool:
    """
    Kill EVERY ``roslaunch`` process on this host via ``killall -9``.

    Host-scoped: not limited to this script's rosmaster. Renamed from
    ``kill_all_roslaunch_process`` to make the host-wide scope
    explicit. The old name is kept as a DeprecationWarning-emitting
    alias for backwards compatibility.

    Returns:
        bool: True if the kill command ran.
    """

    term_cmd = "killall -9 roslaunch"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    rospy.logdebug("Killed all host-wide roslaunch processes.")

    return True


def kill_all_roslaunch_process() -> bool:
    """
    DEPRECATED. Use :func:`kill_all_host_roslaunch_processes` instead.
    """
    warnings.warn(
        "kill_all_roslaunch_process() is deprecated; use "
        "kill_all_host_roslaunch_processes() instead. The behaviour is "
        "unchanged but the name now makes the host-wide scope explicit.",
        DeprecationWarning, stacklevel=2,
    )
    return kill_all_host_roslaunch_processes()


def kill_all_ros_nodes(ros_port=None) -> bool:
    """
    Function to kill all running ROS nodes of the specified or current Rosmaster.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if all nodes were killed, False otherwise.
    """

    if ros_port is not None:
        change_ros_master(ros_port=ros_port)

    term_cmd = "rosnode kill -a"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    rospy.logdebug("Successfully killed all the active nodes instances!")

    # Remove from diagnostic port log
    if ros_port is not None:
        _remove_from_port_log(ros_port)

    return True


def kill_ros_node(node_name, ros_port=None) -> bool:
    """
    Function to kill a ROS node of the given or current Rosmaster.

    Args:
        node_name (str): Name of the node to kill.
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the node was killed, False otherwise.
    """

    if ros_port is not None:
        change_ros_master(ros_port=ros_port)

    term_cmd = "rosnode kill " + node_name
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    rospy.logdebug(f"Successfully killed the node: {node_name}!")

    # Remove from diagnostic port log
    if node_name == "/rosout" and ros_port is not None:
        _remove_from_port_log(ros_port)

    return True


def ros_kill_master(ros_port) -> bool:
    """
    Function to kill a ROS master.

    Args:
        ros_port (str): The ROS_MASTER_URI port.

    Returns:
        bool: True if the master was killed, False otherwise.
    """

    term_cmd = "pkill -f 'roscore -p " + ros_port + "'"
    # subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    result = os.system(term_cmd)
    time.sleep(0.5)

    # rospy.logdebug(f"Successfully killed ROS master: {ros_port}!")

    # Remove the ros master from the ROS_MASTER port list
    # remove_from_rosmaster_list(ros_port)
    #
    # return True

    if result == 0:
        rospy.logdebug(f"Successfully killed ROS master: {ros_port}!")
        # Remove from diagnostic port log
        _remove_from_port_log(ros_port)
        return True
    else:
        rospy.logdebug(f"Failed to kill ROS master: {ros_port}!")
        return False


def clean_ros_logs() -> bool:
    """
    Function to clean all the ros logs.

    Returns:
        bool: True if all the ros logs were closed.
    """

    term_cmd = "rosclean purge -y"
    # subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    # rospy.logdebug("successfully cleaned all the ROS logs")
    # return True

    result = os.system(term_cmd)

    if result == 0:
        rospy.logdebug("Successfully cleaned all the ROS logs")
        return True
    else:
        rospy.logdebug("Failed to clean ROS logs")
        return False


def source_workspace(abs_path) -> bool:
    """
    DEPRECATED no-op.

    This function used to call ``os.system("source devel/setup.bash")``
    against the workspace path. That cannot work for two reasons:

    1. ``os.system`` runs the command in ``/bin/sh`` (commonly ``dash``
       on Ubuntu), where ``source`` is not a builtin — only ``.``
       ("dot-source") is. So the source call fails immediately with
       "sh: 1: source: not found".
    2. Even if it succeeded, sourcing in a child shell only mutates
       the child's environment. The child dies with the subprocess
       and the calling Python process is unaffected.

    To use a ROS workspace from a Python process, source it in the
    shell that launches Python (i.e. ``source devel/setup.bash`` in
    the terminal, then run ``python``).

    This function is preserved for backwards compatibility but is now
    a no-op that emits a DeprecationWarning and a rospy logwarn. It
    will be removed in a future release.

    Args:
        abs_path (str): Unused. Kept for signature compatibility.

    Returns:
        bool: Always False.
    """
    import warnings
    msg = (
        "source_workspace() cannot mutate the calling Python process's "
        "environment. Source the ROS workspace in the shell that "
        "launches Python instead. This function is a no-op and will "
        "be removed in a future release."
    )
    warnings.warn(msg, DeprecationWarning, stacklevel=2)
    rospy.logwarn(msg)
    return False


def ros_launch_launcher(pkg_name=None, launch_file_name=None, launch_file_abs_path=None, args=None,
                        launch_new_term=True, ros_port=None) -> bool:
    """
    Function to execute a roslaunch with args.

    Args:
        pkg_name (str): Name of the package where the launch file is located.
        launch_file_name (str): Name of the launch file.
        launch_file_abs_path (str): Absolute path of the launch file
        args (list of str): Args to pass to the launch file.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the launch file was launched and False otherwise.
    """

    # change the rosmaster
    if ros_port is not None:
        change_ros_master(ros_port=ros_port)

    term_cmd = construct_roslaunch_command(pkg_name, launch_file_name, launch_file_abs_path)

    if term_cmd is None:
        print("Launch Failed! Requires either the absolute path or the pkg_name and the launch_file_name as input!")
        return False

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = "xterm -e ' " + term_cmd + "'"

    subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)

    return True


# helper fn for ros_launch_launcher
def construct_roslaunch_command(pkg_name, launch_file_name, launch_file_abs_path):
    """
    Constructs a roslaunch command using either a package name and launch file name or an absolute path to a launch file.

    Args:
        pkg_name (str): Name of the package where the launch file is located.
        launch_file_name (str): Name of the launch file.
        launch_file_abs_path (str): Absolute path of the launch file

    Returns:
        str: The constructed roslaunch command or None if no valid inputs were provided.
    """

    # roslaunch from package
    if pkg_name and launch_file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug("Package found!")
        except rospkg.common.ResourceNotFound:
            rospy.logerr("Package NOT FOUND! Recheck the Package name and Source the proper ROS Workspace!")
            return None

        file_path = os.path.join(pkg_path, "launch", launch_file_name)
        if os.path.exists(file_path) is False:
            print(f"Launch file {launch_file_name} in {file_path} does not exist!")
            return None

        return f"roslaunch {pkg_name} {launch_file_name}"

    # or roslaunch from a path
    elif launch_file_abs_path is not None:
        if os.path.exists(launch_file_abs_path) is False:
            print(f"Launch file {launch_file_abs_path} does not exist!")
            return None

        return f"roslaunch {launch_file_abs_path}"

    else:
        return None


def ros_node_launcher(pkg_name, node_name, launch_master=False, launch_new_term=True, name=None, ns="/", output="log",
                      ros_port=None, args=None) -> Tuple[str, bool]:
    """
    Function to launch a ROS node from a package. If "launch_master" is "True", it will also launch a ROSCORE with the
    given "ros_port" or with a random ROS master port if "ros_port" is not specified.

    Args:
        pkg_name (str): Name of the package to launch the node from.
        node_name (str): Name of the node to launch.
        launch_master (bool): If ROSMASTER is not running launch it.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).
        name (str): Name to give the node to be launched.
        ns (str): Namespace to give the node to be launched.
        output (str): log, screen, or None
        ros_port (str): The ROS_MASTER_URI port (optional).
        args (List[str]): List of additional arguments to pass to the rosrun command.

    Returns:
        tuple[str, bool]: ROS Master port, and True if the node was launched successfully,
                               False otherwise.
    """

    # change the rosmaster
    if ros_port is not None and not launch_master:
        change_ros_master(ros_port=ros_port)

    # verify if the given package is available
    if not check_package_exists(pkg_name):
        return "", False

    # only if launch_master is True
    rs_port = ""

    # launching the roscore
    if launch_master:
        print("Launching ROS Master")
        if ros_port is not None:
            rs_port = launch_roscore(port=int(ros_port))
        else:
            rs_port = launch_roscore()

    # check if the rosmaster is running
    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("ROS Master not running!")
        return rs_port, False
    else:
        print("ROS Master is running!")

    term_cmd = construct_rosrun_command(pkg_name, node_name, name=name, ns=ns, output=output)

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = f"xterm -e '{term_cmd}'"

    subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)

    return rs_port, True


# helper fn for ros_node_launcher
def construct_rosrun_command(pkg_name, node_name, name=None, ns="/", output="log"):
    """
    Constructs a rosrun command using a package name and node name.

    Args:
        pkg_name (str): Name of the package to launch the node from.
        node_name (str): Name of the node to launch.
        name (str): Name to give the node to be launched.
        ns (str): Namespace to give the node to be launched.
        output (str): log, screen, or None

    Returns:
        str: The constructed rosrun command.
    """

    term_cmd = f"rosrun {pkg_name} {node_name}"

    if name is not None:
        term_cmd += f" __name:={name}"

    term_cmd += f" __ns:={ns}"
    term_cmd += f" __log:={output}"

    return term_cmd


# # helper fn for ros_node_launcher
def check_package_exists(pkg_name):
    """
    Checks if a given package exists.

    Args:
        pkg_name (str): Name of the package.

    Returns:
        bool: True if the package exists and False otherwise.
    """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...!")
        return True
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND! Recheck the Package name and Source the proper ROS Workspace!")
        return False


def ros_load_yaml(pkg_name=None, file_name=None, file_abs_path=None, ns='/', ros_port=None) -> bool:
    """
    Fetch a YAML file from a package or an abs path and load it into the ROS Parameter Server.

    Args:
        pkg_name (str): name of package.
        file_name (str): name of file.
        file_abs_path (str): Absolute path of the YAML file.
        ns (str): namespace to load parameters into.
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if file was loaded, false otherwise.
    """

    # Change the rosmaster if ros_port is not None
    if ros_port is not None:
        change_ros_master(ros_port=ros_port)

    # If pkg_name and file_name are not None, try to locate the package and check if the YAML file exists within it
    if pkg_name and file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug(f"Package {pkg_name} located.")
        except rospkg.common.ResourceNotFound:
            rospy.logerr(f"Package {pkg_name} not found.")
            return False

        file_abs_path = pkg_path + "/config/" + file_name
        if os.path.exists(pkg_path + "/config/" + file_name) is False:
            print(f"Config file {file_name} in {file_abs_path} does not exist")
            return False

    # If pkg_name and file_name are both None but file_abs_path is not None,
    # check if the YAML file exists at the given absolute path
    elif file_abs_path is not None:
        if os.path.exists(file_abs_path) is False:
            print(f"Config file {file_abs_path} does not exist!")
            return False

    # If none of these conditions are met, return False
    else:
        print("Load Failed! Requires either the absolute path or the pkg_name and the file_name as input!")
        return False

    # Load the parameters from the YAML file and upload them to the ROS Parameter Server under the given namespace
    param_list = rosparam.load_file(file_abs_path)
    for params, namespace in param_list:
        rosparam.upload_params(ns, params)

    return True


def load_urdf(model_path=None, pkg_name=None, file_name=None, folder="/urdf", ns=None, args_xacro=None,
              param_name=None, ros_port=None) -> Tuple[bool, Union[str, None]]:
    """
    Function to load a URDF from a ROS package to the parameter server or a string containing the processed URDF data.

    Args:
        model_path (str): The path to the URDF file. Defaults to None.
        pkg_name (str): The name of the ROS package containing the URDF file. Defaults to None.
        file_name (str): The name of the URDF file. Defaults to None.
        folder (str): The folder within the ROS package containing the URDF file. Defaults to "/urdf".
        ns (str): The namespace to use when adding the URDF to the parameter server. Defaults to None.
        args_xacro (list): Additional arguments to pass to xacro when processing the URDF file. Defaults to None.
        param_name (str): The name of the parameter to use when adding the URDF to the parameter server.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[bool, Union[str, None]]: A tuple containing a boolean value indicating whether the loading was
                                        successful and a string containing the processed URDF data. If an error
                                        occurred while loading or processing the URDF file, the first element of
                                        the tuple will be False and the second element will be None.
    """

    # Change the rosmaster
    if ros_port is not None:
        change_ros_master(ros_port=ros_port)

    # Determine the path of the URDF file
    if pkg_name and file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug(f"Package '{pkg_name}' found at path '{pkg_path}'")
        except rospkg.common.ResourceNotFound:
            rospy.logerr(f"Package '{pkg_name}' not found")
            return False, None

        urdf_path = pkg_path + folder + "/" + file_name

    elif model_path is not None:
        urdf_path = model_path
    else:
        rospy.logerr("Invalid input: model_path and pkg_name/file_name are both None")
        return False, None

    # Check if the URDF file exists
    if os.path.exists(urdf_path) is False:
        rospy.logerr(f"Model path '{urdf_path}' does not exist")
        return False, None

    # Process the URDF file using xacro
    list_args = [urdf_path]
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent=' ', **encoding)

    # Add the processed URDF data to the parameter server
    if param_name is not None:

        if ns is not None and ns != "/":
            ns = ns.rstrip('/')
            final_param_name = ns + "/" + param_name
            rospy.logdebug(f"URDF data added to parameter server with name '{final_param_name}' in namespace '{ns}'")
        else:
            final_param_name = param_name
            rospy.logdebug(f"URDF data added to parameter server with name '{final_param_name}'")

        rospy.set_param(final_param_name, model_string)

    return True, model_string


def is_roscore_running() -> bool:
    """
    Function to check if a roscore is currently running.

    Returns:
        bool: True if a roscore is running, False otherwise.
    """

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        rospy.logerr("ROS Master not running!")
        return False
    else:
        rospy.loginfo("ROS Master is running!")
        return True


def change_ros_master(ros_port: str) -> bool:
    """
    Function to set the current ROS Master Environment Variable.

    Args:
        ros_port (str): The ROS_MASTER_URI port.

    Returns:
        bool: True if ROS Master Environment Variables are set to new values.
    """

    # Temporarily assigning values for the ROS_MASTER_URI environment variable
    os.environ["ROS_MASTER_URI"] = f"http://localhost:{ros_port}"

    rospy.logdebug(f"Changed ROS_MASTER_URI to port: {ros_port}")

    return True


def change_ros_master_multi_device(remote_ip: str, local_ip: str, remote_ros_port: str = '11311') -> bool:
    """
    Function to set the current ROS Master Environment Variable for multi-device communication.

    Args:
        remote_ip (str): The remote IP address.
        local_ip (str): The local IP address.
        remote_ros_port (str): The remote ROS_MASTER_URI port. Defaults ros port 11311.

    Returns:
        bool: True if ROS Master Environment Variables are set to new values.
    """

    # Temporarily assigning values for the ROS_MASTER_URI environment variable
    os.environ["ROS_MASTER_URI"] = f"http://{remote_ip}:{remote_ros_port}"

    # Temporarily assigning values for the ROS_HOSTNAME environment variable
    os.environ["ROS_HOSTNAME"] = f"{local_ip}"

    rospy.logdebug(f"Changed ROS_MASTER_URI to: http://{remote_ip}:{remote_ros_port}")

    return True


def init_robot_state_publisher(ns: str = "/", max_pub_freq: float = None, launch_new_term: bool = False) -> bool:
    """
    Function to initialize the robot state publisher.

    Args:
        ns (str): Namespace to give the node to be launched.
        max_pub_freq (float): Maximum frequency at which to publish the robot state.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).

    Returns:
        bool: True if the node was launched successfully, False otherwise.
    """

    if max_pub_freq is not None:
        _, launch_done = ros_node_launcher(pkg_name="robot_state_publisher",
                                           node_name="robot_state_publisher",
                                           launch_new_term=launch_new_term,
                                           ns=ns,
                                           args=[f"publish_frequency:={max_pub_freq}"]
                                           )
    else:
        # we don't need the first two since we are not launching a new roscore
        _, launch_done = ros_node_launcher(pkg_name="robot_state_publisher",
                                           node_name="robot_state_publisher",
                                           launch_new_term=launch_new_term,
                                           ns=ns)

    if launch_done:
        rospy.logdebug("Successfully initialised the Robot State Publisher!")
    else:
        rospy.logerr("Robot State Publisher initialisation Failed!")

    return launch_done


def remove_all_from_rosmaster_list() -> bool:
    """
    DEPRECATED. Truncates the diagnostic port log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "remove_all_from_rosmaster_list() is deprecated and now only "
        "truncates a diagnostic log. Port allocation is handled by "
        "socket.bind(0) inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    try:
        if os.path.exists(_PORT_LOG_PATH):
            with open(_PORT_LOG_PATH, 'w') as f:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    f.write('')
                finally:
                    try:
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                    except OSError:
                        pass
    except OSError as e:
        rospy.logwarn(f"Could not truncate port log {_PORT_LOG_PATH}: {e}")
        return False
    return True
