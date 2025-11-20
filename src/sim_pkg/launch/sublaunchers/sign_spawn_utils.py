import importlib.util
import os
import sys
from pathlib import Path
from typing import Iterable, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

_CURRENT_DIR = Path(__file__).resolve().parent
if str(_CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(_CURRENT_DIR))

_DATA_SPEC = importlib.util.spec_from_file_location(
    "_sign_spawn_data", _CURRENT_DIR / "_sign_spawn_data.py"
)
_DATA_MODULE = importlib.util.module_from_spec(_DATA_SPEC)
assert _DATA_SPEC.loader is not None
_DATA_SPEC.loader.exec_module(_DATA_MODULE)
SIGN_GROUPS = _DATA_MODULE.SIGN_GROUPS


def _create_spawn_node(model_file: str, *, name: str, x: float, y: float,
                       z: float, roll: float, pitch: float, yaw: float) -> Node:
    models_pkg = get_package_share_directory("models_pkg")
    sdf_path = os.path.join(models_pkg, model_file)
    arguments = [
        "-file", sdf_path,
        "-entity", name,
        "-x", f"{x}",
        "-y", f"{y}",
        "-z", f"{z}",
        "-R", f"{roll}",
        "-P", f"{pitch}",
        "-Y", f"{yaw}",
    ]
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=name,
        output="screen",
        arguments=arguments,
    )


def generate_spawn_launch_description(
    model_keys: Iterable[str],
    *,
    initial_delay: float = 1.0,
    step_delay: float = 0.4,
) -> LaunchDescription:
    actions: List[TimerAction] = []
    delay = initial_delay
    for key in model_keys:
        signs = SIGN_GROUPS.get(key, [])
        for spec in signs:
            node = _create_spawn_node(
                key,
                name=spec["name"],
                x=spec["x"],
                y=spec["y"],
                z=spec["z"],
                roll=spec["roll"],
                pitch=spec["pitch"],
                yaw=spec["yaw"],
            )
            actions.append(TimerAction(period=delay, actions=[node]))
            delay += step_delay
    ld = LaunchDescription()
    for action in actions:
        ld.add_action(action)
    return ld
