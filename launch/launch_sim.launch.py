## Launch file for the simulation of the robot in Gazebo and RViz


import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled


    package_name='test_robot'
    
    rviz_node = IncludeLaunchDescription( #launches rviz, robot_state_publisher and joint_state_publisher 
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('test_robot'),'launch'),
         '/test_robot.launch.py'])
    )
    gz_node = IncludeLaunchDescription( #launches gazebo with ros stuff
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros_gz_sim'),'launch'),
         '/gz_sim.launch.py']), launch_arguments={'use_sim_time': 'True', 'gz_args': 'shapes.sdf',  }.items()
    )
    spawn_robot = IncludeLaunchDescription( #launches the robot in gazebo
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('ros_gz_sim'),'launch'),
         '/gz_spawn_model.launch.py']),launch_arguments={'topic': 'robot_description'}.items()
    )
    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                      '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            parameters=[{'qos_overrides.subscriber.reliability': 'reliable',
                        }],
            output='screen'
        )


    # Launch them all!
    return LaunchDescription([
        rviz_node,
        gz_node,
        bridge,
        spawn_robot
    ])
