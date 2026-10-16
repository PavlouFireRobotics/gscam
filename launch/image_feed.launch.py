"""Launch one camera decoder and share its ROS images with local consumers."""

import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_parameters(camera, decoder):
    if not isinstance(camera, dict):
        raise ValueError('Camera settings must be a mapping')
    if decoder not in ('rockchip', 'software'):
        raise ValueError('decoder must be rockchip or software')
    codec = camera.get('codec', 'h265')
    if codec not in ('h264', 'h265'):
        raise ValueError('feed.codec must be h264 or h265')
    protocols = camera.get('protocols', 'tcp')
    if protocols not in ('tcp', 'udp'):
        raise ValueError('feed.protocols must be tcp or udp')
    latency = camera.get('latency', 150)
    if type(latency) is not int or latency < 0:
        raise ValueError('feed.latency must be a non-negative integer')
    url = camera['rtsp_url']
    if not isinstance(url, str) or not url.startswith(('rtsp://', 'rtsps://')):
        raise ValueError('feed.rtsp_url must be an RTSP URL')
    # Quote for GStreamer syntax; this string is never passed through a shell.
    url = url.replace('\\', '\\\\').replace('"', '\\"')
    drop = str(camera.get('drop_on_latency', True)).lower()
    pipeline = (
        f'rtspsrc location="{url}" protocols={protocols} '
        f'latency={latency} drop-on-latency={drop} '
        f'! rtp{codec}depay ! {codec}parse '
        f'! video/x-{codec},stream-format=byte-stream,alignment=au '
    )
    # gscam adds the sink later; use an element rather than bare trailing caps.
    if decoder == 'rockchip':
        pipeline += '! mppvideodec format=RGB ! capsfilter caps=video/x-raw,format=RGB'
    else:
        pipeline += f'! avdec_{codec} ! videoconvert ! capsfilter caps=video/x-raw,format=RGB'
    info_url = camera.get('camera_info_url', '')
    if not isinstance(info_url, str):
        raise ValueError('feed.camera_info_url must be a string')
    return {
        'gscam_config': pipeline,
        'image_encoding': 'rgb8',
        'frame_id': camera['frame_id'],
        'camera_name': camera.get('camera_name', camera['feed_name']),
        'camera_info_url': info_url,
        'sync_sink': False,
        'reopen_on_eof': False,
        'use_gst_timestamps': camera.get('use_gst_timestamps', False),
        'use_sensor_data_qos': camera.get('use_sensor_data_qos', True),
        'use_image_transport': camera.get('use_image_transport', False),
        'sample_timeout_sec': float(camera.get('sample_timeout_sec', 5.0)),
        'startup_timeout_sec': float(camera.get('startup_timeout_sec', 5.0)),
        # Humble recognizes this allowlist. Foxy raw-only mode bypasses plugins.
        'camera.image_raw.enable_pub_plugins': [
            'image_transport/raw',
            'image_transport/compressed',
        ],
    }


def launch_camera(context):
    path = LaunchConfiguration('feed_config').perform(context)
    with open(path, encoding='utf-8') as handle:
        camera = yaml.safe_load(handle)
    if not isinstance(camera, dict):
        raise ValueError('Feed configuration must be a mapping')
    if 'defaults' in camera or 'feeds' in camera:
        raise ValueError('Configure one feed directly; defaults and feeds are no longer supported')

    name = camera.get('feed_name')
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
        raise ValueError('feed_name must be a valid ROS name using letters, digits and underscores')
    decoder_override = LaunchConfiguration('decoder').perform(context)
    decoder = decoder_override or camera.get('decoder', 'rockchip')
    return [
        Node(
            package='gscam',
            executable='gscam_node',
            name=f'{name}_images',
            namespace=LaunchConfiguration('namespace').perform(context)
            or camera.get('namespace', ''),
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[camera_parameters(camera, decoder)],
            remappings=[
                ('camera/image_raw', f'{name}/image_raw'),
                ('camera/image_raw/compressed', f'{name}/image_raw/compressed'),
                ('camera/camera_info', f'{name}/camera_info'),
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'feed_config',
                description='Path to a single-camera RTSP feed YAML configuration',
            ),
            DeclareLaunchArgument(
                'decoder',
                default_value='',
                description='Override configured decoder: rockchip or software',
            ),
            DeclareLaunchArgument(
                'namespace', default_value='', description='Override the configured ROS namespace'
            ),
            OpaqueFunction(function=launch_camera),
        ]
    )
