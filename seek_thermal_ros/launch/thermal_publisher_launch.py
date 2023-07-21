import launch
from launch import LaunchDescription, SomeSubstitutionsType
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_packages_with_prefixes


def generate_launch_description():
    colorPalette = LaunchConfiguration("colorPalette", default="TYRIAN")
    rotationValue = LaunchConfiguration("rotationValue", default=270)
    greyscale = LaunchConfiguration("greyscale", default=False)

    launch_description_nodes = []

    seek_node = Node(
        package="seek_thermal_ros",
        executable="thermal_publisher",
        name="thermal_publisher",
        parameters=[
            {"colorPalette": colorPalette},
            {"rotationValue": rotationValue},
            {"greyscale": greyscale},
        ],
    )
    launch_description_nodes.append(seek_node)

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
    launch_description_nodes.append(compressed_seek_node)

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
        launch_description_nodes.append(ffmpeg_seek_node)

    # This action will kill all nodes once seek_node has exited
    the_killer = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=seek_node,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    )
    launch_description_nodes.append(the_killer)

    return LaunchDescription(launch_description_nodes)
