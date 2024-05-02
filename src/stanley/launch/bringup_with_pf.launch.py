from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Bringup
    f1tenth_stack_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('f1tenth_stack'), 
                 'launch', 
                 'bringup_launch.py']
            )
        )
    )

    # pf
    particle_filter_localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('particle_filter'), 
                 'launch', 
                 'localize_launch.py']
            )
        )
    )

    bringup_with_pf_launch = GroupAction([
        PushRosNamespace('ego'),
        f1tenth_stack_bringup_launch,
        particle_filter_localize_launch
    ])

    return LaunchDescription([
        bringup_with_pf_launch
    ])