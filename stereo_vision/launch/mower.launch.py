from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

# stereo_vision is copied into /app as plain files (not a colcon-built ROS package),
# so all paths are resolved relative to this launch file.
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def generate_launch_description():
    """Build the robot TF and stereo nodes from the shared hardware configuration."""

    urdf_file = os.path.join(APP_DIR, 'urdf', 'OmniMower.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Load chassis and camera specifications from environment variables (from omnimow.env) with robust fallbacks [DYNAMIC LOADING]
    camera_x = os.getenv('OMNIMOW_CAMERA_OFFSET_X', '0.25')
    camera_z = os.getenv('OMNIMOW_CAMERA_HEIGHT_Z', '0.10')
    camera_pitch = os.getenv('OMNIMOW_CAMERA_PITCH_Y', '0.0')

    # Baseline and focal length come from the stereo calibration (calibrate_stereo.py)
    calibration_dir = os.getenv('OMNIMOW_CALIB_DIR', '/calibration')

    # Nav2 (incl. robot radius / inflation from omnimow.env) runs in its own container: navigation/

    robot_desc_formatted = robot_desc.format(
        CAMERA_OFFSET_X=camera_x,
        CAMERA_HEIGHT_Z=camera_z,
        CAMERA_PITCH_Y=camera_pitch
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
        ExecuteProcess(
            cmd=[
                'python3', os.path.join(APP_DIR, 'stereo_node.py'),
                '--ros-args', '-r', '__node:=stereo_node',
                '-p', f'calibration_dir:={calibration_dir}',
                '-p', f'camera_height_z:={float(camera_z)}',
                '-p', f'camera_offset_x:={float(camera_x)}',
                '-p', f'camera_pitch_y:={float(camera_pitch)}',
            ],
            output='screen'
        )
    ])