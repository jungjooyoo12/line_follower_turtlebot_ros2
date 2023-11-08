import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_tb3_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    _pkg_tb3_bringup = FindPackageShare(package='turtlebot3_bringup').find('turtlebot3_bringup')
    
    pkg_share = FindPackageShare(package='follow_line').find('follow_line')
    default_world_path = os.path.join(pkg_share, 'maps', 'lfm2_nowalls.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    gzserver_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': default_world_path}.items()
        )

    gzclient_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        )

    robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()
        )

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'model.rviz')],
            output='screen',
        )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        rviz_cmd,
    ])