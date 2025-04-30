import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('student_dan_pkg')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    simulator_launch = os.path.join(get_package_share_directory('mpc_rbt_simulator'),'launch','simulation.launch.py')

    
    return LaunchDescription([

        #Launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path]
            #parameters=[{'use_sim_time': True}]
        ),

        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulator_launch)
            )
            ]
        ),
        
        TimerAction(
            period=15.0,
            actions=[
                # Launch your node
                Node(
                    package='student_dan_pkg',
                    executable='localization',
                    name='localization',
                    output='screen'
                    #parameters=[{'use_sim_time': True}]
                )            

            ]
        ),

        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='student_dan_pkg',
                    executable='planning',
                    name='planning',
                    output='screen'
                )   
            
            ]
        ),

        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='student_dan_pkg',
                    executable='motioncontrol',
                    name='motioncontrol',
                    output='screen'
                )
            ]
        )  
    ]
    )

