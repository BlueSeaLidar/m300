from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory("pacecat_m300_driver")
    driver = Node(
        package="pacecat_m300_driver", 
        executable="driver",
        output="screen",
        emulate_tty=True, #模拟一个终端环境
        parameters=[os.path.join(share_dir,"params","LDS-M300-E.yaml")]       
    )
    return LaunchDescription([
        driver
    ])
