""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: wrist_3_link -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "wrist_3_link",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0317874",
                "--y",
                "-0.102658",
                "--z",
                "0.0174343",
                "--qx",
                "0.000155118",
                "--qy",
                "-0.00175704",
                "--qz",
                "0.0173445",
                "--qw",
                "0.999848",
                # "--roll",
                # "0.00037114",
                # "--pitch",
                # "-0.00350817",
                # "--yaw",
                # "0.0346914",
            ],
        ),
    ]
    return LaunchDescription(nodes)
