from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    args={'left_follower':'can_left_fol','right_follower':'can_right_fol','left_leader':'can_left_lea','right_leader':'can_right_lea','leader_source':'feedback'}
    params={k:LaunchConfiguration(k) for k in args}
    params['auto_teach']=ParameterValue(LaunchConfiguration('auto_teach'),value_type=bool)
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]+[
        DeclareLaunchArgument('auto_teach',default_value='true'),
        Node(package='piper_aio_ros2',executable='four_can',output='screen',parameters=[params])])
