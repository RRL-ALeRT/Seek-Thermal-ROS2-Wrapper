import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_packages_with_prefixes


def generate_launch_description():
    colorPalette = LaunchConfiguration("colorPalette", default="TYRIAN")
    rotationValue = LaunchConfiguration("rotationValue", default=0)
    greyscale = LaunchConfiguration("greyscale", default=False)
    
    launch_list = []
    
    seek_node = Node(
        package="seek_thermal_ros",
        executable="thermal_publisher",
        name="thermal_publisher",
        parameters=[
            {"colorPalette": colorPalette},
            {"rotationValue": rotationValue},
            {"greyscale": greyscale},
        ],
        respawn=True,  # Automatically respawn the node if it exits
    )
    launch_list.append(seek_node)
    
    compressed_seek_node = Node(
        package="image_transport",
        executable="republish",
        name="thermal_republish",
        arguments=[  
            "raw",
            "compressed"
        ],
        remappings=[
            ("in", "/thermalImage"),
            ("out/compressed","/thermalImage/compressed")
        ]
    )
    launch_list.append(compressed_seek_node)

    # Check if ffmpeg plugin is installed
    if "ffmpeg_image_transport" in get_packages_with_prefixes():
        ffmpeg_seek_node = Node(
            package="image_transport",
            executable="republish",
            name="thermal_republish",
            arguments=[  
                "raw",
                "ffmpeg"
            ],
            remappings=[
                ("in", "/thermalImage"),
                ("out/ffmpeg","/thermalImage/ffmpeg")
            ]
        )
        launch_list.append(ffmpeg_seek_node)

    return LaunchDescription(launch_list)
