import sys
from pathlib import Path

_CURRENT_DIR = Path(__file__).resolve().parent
if str(_CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(_CURRENT_DIR))

from sign_spawn_utils import generate_spawn_launch_description


ALL_GROUPS = [
    "stop_sign/model.sdf",
    "priority_sign/model.sdf",
    "traffic_light/model.sdf",
    "parking_sign/model.sdf",
    "roundabout_sign/model.sdf",
    "crosswalk_sign/model.sdf",
    "enter_highway_sign/model.sdf",
    "leave_highway_sign/model.sdf",
    "oneway_sign/model.sdf",
    "rcCar_assembly_obstacle/model.sdf",
]


def generate_launch_description():
    return generate_spawn_launch_description(ALL_GROUPS)
