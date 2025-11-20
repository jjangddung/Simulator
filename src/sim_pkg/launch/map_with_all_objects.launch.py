import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _include_launch(path, **launch_arguments):
    return IncludeLaunchDescription(
        AnyLaunchDescriptionSource(path),
        launch_arguments={k: v for k, v in launch_arguments.items() if v is not None}.items(),
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("sim_pkg")
    launch_dir = os.path.join(pkg_share, "launch")

    gazebo_launch = os.path.join(launch_dir, "gazebo.launch")
    default_world = os.path.join(pkg_share, "worlds", "world_with_separators.world")

    world_arg = LaunchConfiguration("world")

    # staged_groups = [
        # ["sublaunchers/car.launch", "sublaunchers/traffic_lights.launch"],
        # ["sublaunchers/obstacle_car.launch", "sublaunchers/members.launch",
        #  "sublaunchers/ramp.launch", "sublaunchers/roadblock.launch"],
        # ["sublaunchers/enter_highway_signs.launch", "sublaunchers/leave_highway_signs.launch",
        #  "sublaunchers/prohibited_signs.launch", "sublaunchers/oneway_signs.launch",
        #  "sublaunchers/parking_signs.launch"],
        # ["sublaunchers/crosswalk_signs.launch", "sublaunchers/pedestrian_objects.launch",
        #  "sublaunchers/priority_signs.launch", "sublaunchers/roundabout_signs.launch",
        #  "sublaunchers/stop_signs.launch"],
    # ]

    staged_groups = [
        # ["sublaunchers/spawn_stop_signs.launch.py"],
        # ["sublaunchers/spawn_priority_signs.launch.py"],
        # ["sublaunchers/spawn_traffic_lights.launch.py"],
        ["sublaunchers/spawn_parking_signs.launch.py"],
        # ["sublaunchers/spawn_roundabout_signs.launch.py"],
        # ["sublaunchers/spawn_crosswalk_signs.launch.py"],
        # ["sublaunchers/spawn_enter_highway_signs.launch.py"],
        # ["sublaunchers/spawn_leave_highway_signs.launch.py"],
        # ["sublaunchers/spawn_oneway_signs.launch.py"],
        ["sublaunchers/car.launch", "sublaunchers/wheel_imu_odom.launch", "sublaunchers/robot_localization.launch", "sublaunchers/set_initial_pose.launch"],
        # ["sublaunchers/spawn_obstacles.launch.py"],
    ]

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Path to the Gazebo world file.",
        )
    )

    ld.add_action(
        _include_launch(
            gazebo_launch,
            world=world_arg,
        )
    )

    delay = 5.0
    step = 5.0

    for group in staged_groups:
        actions = [_include_launch(os.path.join(launch_dir, rel_path)) for rel_path in group]
        ld.add_action(TimerAction(period=delay, actions=actions))
        delay += step

    return ld
