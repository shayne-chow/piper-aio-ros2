from setuptools import setup
from glob import glob
setup(name='piper_aio_ros2', version='0.1.0', packages=['piper_aio_ros2'],
 data_files=[('share/ament_index/resource_index/packages',['resource/piper_aio_ros2']),('share/piper_aio_ros2',['package.xml']),('share/piper_aio_ros2/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'], zip_safe=True, maintainer='top', maintainer_email='top@example.com', description='Four CAN teleoperation', license='MIT',
 entry_points={'console_scripts':['four_can = piper_aio_ros2.driver:main','collect = piper_aio_ros2.collector:main']})
