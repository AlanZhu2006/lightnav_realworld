"""Go2 using an existing camera stream, with all host-specific settings explicit."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "server_url": os.environ.get("LIGHTNAV_SERVER_URL", ""),
        "network_interface": os.environ.get("LIGHTNAV_NETWORK_INTERFACE", "enP8p1s0"),
        "image_topic": "/camera/camera/color/image_raw",
        "image_transport": "raw",
        "linear_limit": "0.3",
        "angular_limit": "0.5",
        "web_host": "0.0.0.0",
        "web_port": "8088",
        "recording_directory": os.environ.get(
            "LIGHTNAV_RECORDINGS_DIR",
            os.path.expanduser("~/.local/share/lightnav_realworld/recordings"),
        ),
    }
    declarations = [
        DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()
    ]
    camera = {
        key: ParameterValue(LaunchConfiguration(key), value_type=str)
        for key in ("image_topic", "image_transport")
    }
    linear = ParameterValue(LaunchConfiguration("linear_limit"), value_type=float)
    angular = ParameterValue(LaunchConfiguration("angular_limit"), value_type=float)
    return LaunchDescription(
        declarations
        + [
            Node(
                package="vln_client",
                executable="vln_client",
                name="vln_client",
                parameters=[
                    camera,
                    {
                        "server_url": ParameterValue(
                            LaunchConfiguration("server_url"), value_type=str
                        ),
                    },
                ],
            ),
            Node(
                package="vln_web",
                executable="vln_web",
                name="vln_web",
                parameters=[
                    camera,
                    {
                        "host": ParameterValue(LaunchConfiguration("web_host"), value_type=str),
                        "port": ParameterValue(LaunchConfiguration("web_port"), value_type=int),
                        "manual_linear_limit": linear,
                        "manual_angular_limit": angular,
                        "recording_directory": ParameterValue(
                            LaunchConfiguration("recording_directory"), value_type=str
                        ),
                    },
                ],
            ),
            Node(
                package="vln_mpc",
                executable="vln_mpc",
                name="vln_mpc",
                parameters=[
                    {
                        "track_v_max": linear,
                        "objnav_v_max": linear,
                        "w_max": angular,
                        "v_output_scale": 1.0,
                        "w_output_scale": 1.0,
                    }
                ],
            ),
            Node(
                package="go2_adapter",
                executable="go2_adapter",
                name="go2_adapter",
                parameters=[
                    {
                        "network_interface": ParameterValue(
                            LaunchConfiguration("network_interface"), value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
