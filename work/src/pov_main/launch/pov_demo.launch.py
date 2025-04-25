from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パラメータ
    hef_path = "/home/pi/Work/POV_Demo_Device/work/src/pov_hailo8_node/yolov8cls_rio_classification.hef"

    # 1. pov_hailo8_node
    hailo8_node = Node(
        package="pov_hailo8_node",
        executable="pov_hailo8_node",
        name="pov_hailo8_node",
        parameters=[
            {"hef_file": hef_path}
        ],
        output="screen"
    )

    # 2. bluetooth_central_node
    bluetooth_node = Node(
        package="pov_bluetooth_central",
        executable="bluetooth_central_node",
        name="bluetooth_central_node",
        output="screen"
    )

    # 3. realsense2_camera のlaunch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py"
            )
        ]),
        launch_arguments={
            "enable_gyro": "true",
            "enable_accel": "true"
        }.items()
    )

    return LaunchDescription([
        hailo8_node,
        bluetooth_node,
        realsense_launch
    ])