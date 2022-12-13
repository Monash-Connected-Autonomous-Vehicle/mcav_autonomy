#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
import xacro
import launch
import launch_ros.actions


def generate_launch_description():
    # Load the URDF into a parameter
    bringup_dir = get_package_share_directory('mcav_sd_model')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'sd_twizy.urdf.xacro')
    robot_description = {"robot_description": xacro.process_file(xacro_path).toxml()}

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
        ),
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()