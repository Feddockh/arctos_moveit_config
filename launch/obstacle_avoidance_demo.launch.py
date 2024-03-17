from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Find the package where the launch files are located
    realsense2_camera_path = FindPackageShare('realsense2_camera')
    arctos_moveit_config_path = FindPackageShare('arctos_moveit_config')

    return LaunchDescription([

        # Include the RealSense launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense2_camera_path, '/launch/rs_launch.py']),
            launch_arguments = {
                'camera_name':                      'camera',
                'camera_namespace':                 'camera',
                'accelerate_gpu_with_glsl':         'false',
                'output':                           'screen',
                'enable_color':                     'true',
                'enable_depth':                     'true',
                'depth_module.enable_auto_exposure':'true',
                'enable_sync':                      'false',
                'clip_distance':                    '-2.0',
                'publish_tf':                       'true',
                'tf_publish_rate':                  '14.0',
                'pointcloud.enable':                'true',
                'pointcloud.stream_filter':         '2',
                'pointcloud.stream_index_filter':   '0',
                'pointcloud.ordered_pc':            'false',
                'align_depth.enable':               'false',
                'colorizer.enable':                 'false',
                'decimation_filter.enable':         'true',
                'spatial_filter.enable':            'true',
                'temporal_filter.enable':           'true',
                'disparity_filter.enable':          'false',
                'hole_filling_filter.enable':       'false',
                'hdr_merge.enable':                 'false',
                'wait_for_device_timeout':          '-1.0',
                'reconnect_timeout':                '6.0',
            }.items(),
        ),        

        # Set parameter for realsense to moveit node
        Node(
            package='arctos_moveit_config',
            executable='realsense_to_moveit',
            name='realsense_to_moveit',
            output='screen'
        ),

        # Include the demo.launch.py file from the arctos_moveit_config package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([arctos_moveit_config_path, '/launch/demo.launch.py']),
        ),

        # Give transform from base_link to camera_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='to_camera',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        #     output='screen'
        # ),
    ])