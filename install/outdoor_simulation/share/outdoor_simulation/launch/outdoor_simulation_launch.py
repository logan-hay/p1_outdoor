#!/usr/bin/env python

# Necessary imports
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
import launch


def generate_launch_description():
    package_dir = get_package_share_directory('outdoor_simulation')
    world = LaunchConfiguration('world')

    # Launch Webots simulation
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='outside.wbt',
            description='Choose one of the world files from the `/indoor_simulation/worlds` directory'
        ),
        webots,
        webots._supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
