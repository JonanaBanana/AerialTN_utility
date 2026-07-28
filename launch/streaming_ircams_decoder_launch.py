from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('aerial_tn_utility')

    return LaunchDescription([
        Node(
            package='aerial_tn_utility',
            executable='ircam_decoder',
            name='decoder_ircam_node_wide',
            parameters=[{
            'input_topic': '/ircam_wide/h264',
            'output_topic': '/ircam_wide/decoded'
            }]
        ),
        Node(
        package='aerial_tn_utility',
        executable='ircam_decoder',
        name='decoder_ircam_node_narrow',
        parameters=[{
            'input_topic': '/ircam_narrow/h264',
            'output_topic': '/ircam_narrow/decoded'
            }]
        ),
        Node(
            package='aerial_tn_utility',
            executable='px4_odom_republisher_node',
            name='px4_odom_republisher_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'ircams.rviz')]
        ),
    ])
    