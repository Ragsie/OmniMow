from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Build the robot TF and stereo nodes from the shared hardware configuration."""

    urdf_file = os.path.join(
        get_package_share_directory('stereo_vision'),
        'urdf',
        'OmniMower.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Load chassis and camera specifications from environment variables (from omnimow.env) with robust fallbacks [DYNAMIC LOADING]
    camera_x = os.getenv('OMNIMOW_CAMERA_OFFSET_X', '0.25')
    camera_z = os.getenv('OMNIMOW_CAMERA_HEIGHT_Z', '0.10')
    camera_pitch = os.getenv('OMNIMOW_CAMERA_PITCH_Y', '0.0')
    gps_x = os.getenv('OMNIMOW_GPS_OFFSET_X', '-0.15')
    gps_z = os.getenv('OMNIMOW_GPS_HEIGHT_Z', '0.25')

    device_left = os.getenv('OMNIMOW_CAMERA_DEVICE_LEFT', '/dev/video0')
    device_right = os.getenv('OMNIMOW_CAMERA_DEVICE_RIGHT', '/dev/video1')

    baseline = float(os.getenv('OMNIMOW_CAMERA_BASELINE', '0.06'))
    focal_length = float(os.getenv('OMNIMOW_CAMERA_FOCAL_LENGTH', '350.0'))

    # Dynamic Nav2 parameter injection for the asymmetric cutter head.
    # To prevent the asymmetric cutter head from scraping against obstacles,
    # we load the Nav2 yaml template and replace robot_radius and inflation_radius
    # with values directly from omnimow.env before running.
    nav2_template_file = os.path.join(
        get_package_share_directory('stereo_vision'),
        'config',
        'nav2_params.yaml'
    )
    resolved_nav2_file = '/tmp/nav2_params_resolved.yaml'

    if os.path.exists(nav2_template_file):
        with open(nav2_template_file, 'r') as f:
            nav2_template = f.read()

        # Resolve placeholders using the dimensions selected for this mower.
        nav2_resolved = nav2_template.format(
            ROBOT_RADIUS=os.getenv('OMNIMOW_ROBOT_RADIUS', '0.28'),
            INFLATION_RADIUS=os.getenv('OMNIMOW_NAV2_INFLATION_RADIUS', '0.48')
        )

        # Save the finished runtime configuration for Nav2
        with open(resolved_nav2_file, 'w') as f:
            f.write(nav2_resolved)
        print(f"[OmniMow Launch] Dynamic Nav2 configuration saved in {resolved_nav2_file} with inflation={os.getenv('OMNIMOW_NAV2_INFLATION_RADIUS', '0.48')}m")

    robot_desc_formatted = robot_desc.format(
        CAMERA_OFFSET_X=camera_x,
        CAMERA_HEIGHT_Z=camera_z,
        CAMERA_PITCH_Y=camera_pitch,
        GPS_OFFSET_X=gps_x,
        GPS_HEIGHT_Z=gps_z
    )

    return LaunchDescription([
        # Publish robot transformations (TF-tree with dynamically formatted URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc_formatted}]
        ),
        # Start the central 3D Stereo Vision node (with parameters from omnimow.env) [MIGRATED TO DUAL CAM]
        Node(
            package='stereo_vision',
            executable='stereo_node.py',
            name='stereo_node',
            output='screen',
            parameters=[{
                'video_device_left': device_left,
                'video_device_right': device_right,
                'frame_id': 'camera_link',
                'baseline': baseline,
                'focal_length': focal_length,
                'camera_height_z': float(camera_z),
                'camera_offset_x': float(camera_x),
                'camera_pitch_y': float(camera_pitch)
            }]
        )
    ])