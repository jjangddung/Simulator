import sys
from pathlib import Path

_CURRENT_DIR = Path(__file__).resolve().parent
if str(_CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(_CURRENT_DIR))

from sign_spawn_utils import generate_spawn_launch_description


def generate_launch_description():
    return generate_spawn_launch_description(["stop_sign/model.sdf"])
