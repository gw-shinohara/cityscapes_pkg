from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    cityscape_streamerノードを起動するためのLaunchDescriptionを生成します。
    """

    dataroot_arg = DeclareLaunchArgument(
        'dataroot',
        default_value='/root/data/cityscapes',
        description='Path to the Cityscapes dataroot.'
    )

    scene_names_arg = DeclareLaunchArgument(
        'scene_names',
        default_value="['aachen', 'bochum']",
        description='List of Cityscapes scenes (cities) to stream.'
    )

    play_all_scenes_arg = DeclareLaunchArgument(
        'play_all_scenes',
        default_value='False',
        description='If True, plays all scenes and ignores scene_names.'
    )

    rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='10.0',
        description='Publishing rate in Hz.'
    )

    cityscape_streamer_node = Node(
        package='cityscape_pkg',
        executable='cityscape_streamer',
        name='cityscape_streamer',
        output='screen',
        parameters=[{
            'dataroot': LaunchConfiguration('dataroot'),
            'scene_names': LaunchConfiguration('scene_names'),
            'play_all_scenes': LaunchConfiguration('play_all_scenes'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }]
    )

    return LaunchDescription([
        dataroot_arg,
        scene_names_arg,
        play_all_scenes_arg,
        rate_arg,
        cityscape_streamer_node
    ])

