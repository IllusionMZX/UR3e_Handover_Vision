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
                "-0.0339201",
                "--y",
                "-0.102909",
                "--z",
                "0.021975",
                "--qx",
                "0.000391327",
                "--qy",
                "9.93047e-05",
                "--qz",
                "0.0210168",
                "--qw",
                "0.999779",
                # "--roll",
                # "0.000778307",
                # "--pitch",
                # "0.000215014",
                # "--yaw",
                # "0.0420366",
            ],
        ),
    ]
    return LaunchDescription(nodes)
