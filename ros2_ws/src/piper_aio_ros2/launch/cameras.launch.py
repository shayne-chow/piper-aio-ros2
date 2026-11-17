"""Three Orbbec cameras; require explicit model launch and USB/serial mappings."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def cameras(context):
    model=LaunchConfiguration('camera_launch').perform(context)
    if os.path.basename(model)!=model or not model.endswith('.launch.py'): raise ValueError('camera_launch must be a model launch filename')
    path=os.path.join(get_package_share_directory('orbbec_camera'),'launch',model)
    if not os.path.isfile(path): raise ValueError(f'Unknown model launch: {path}')
    serials=[LaunchConfiguration(f'{side}_serial').perform(context) for side in ('front','left','right')]
    if any(not x for x in serials) or len(set(serials))!=3: raise ValueError('Three distinct camera serial numbers are required')
    return [TimerAction(period=float(i*3),actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments={
        'camera_name':name,'serial_number':serial,'device_num':'3','enable_color':'true',
        'enable_depth':LaunchConfiguration('enable_depth').perform(context),'enable_left_ir':'false','enable_right_ir':'false',
        'color_width':'640','color_height':'480','color_fps':'30','depth_width':'640','depth_height':'480','depth_fps':'30',
    }.items())]) for i,(name,serial) in enumerate(zip(('camera_f','camera_l','camera_r'),serials))]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_launch'),DeclareLaunchArgument('front_serial'),DeclareLaunchArgument('left_serial'),DeclareLaunchArgument('right_serial'),DeclareLaunchArgument('enable_depth',default_value='false'),OpaqueFunction(function=cameras)])
