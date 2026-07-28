from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('aerial_tn_utility')

    return LaunchDescription([
        Node(
            package='aerial_tn',
            executable='voxl_h264_decoder',
            name='decoder_tracking_down_node',
            parameters=[{
                'input_topic': '/tracking_down_misp_encoded',
                'output_topic': '/tracking_down_misp_decoded',
                'frame_id': 'tracking_down',
                'live_stream': True, 
                'convert_to_bgr': True
            }]
        ),
        Node(
            package='aerial_tn',
            executable='voxl_h264_decoder',
            name='decoder_tracking_front_node',
            parameters=[{
                'input_topic': '/tracking_front_misp_encoded',
                'output_topic': '/tracking_front_misp_decoded',
                'frame_id': 'tracking_front',
                'live_stream': True, 
                'convert_to_bgr': True
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
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'voxl.rviz')]
        ),
    ])
    