import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('student_dan_pkg')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    
    return LaunchDescription([
        # Launch your node
        Node(
            package='student_dan_pkg',
            executable='localization',
            name='localization',
            output='screen'
        ),
        
        # Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path]
        )

        # Launch Keyboardcontrol
        #Node(
        #    package='student_dan_pkg',
        #    executable='keyboardcontrol',
        #    name='keyboardcontrol',
        #    output='screen',
        #    parameters=[{'cmd_vel': '/keyboard_input'}] 
        #)
    ])
