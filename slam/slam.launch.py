"""
OmniMow visual SLAM with RTAB-Map (stereo), following the FusionCore + RTAB-Map feedback loop
from rtabmap_ros' fusioncore demo:

  FusionCore (IMU + wheels + visual odometry) -> TF odom -> base_link, /fusion/odom
  stereo_odometry  (rectified stereo, guess from FusionCore's odom) -> /rtabmap/stereo_odometry
                    -> fed back into FusionCore as a second velocity source (encoder2)
  rtabmap SLAM     (stereo + odom TF) -> loop closures on landmarks in the garden -> TF map -> odom

The map is stored in /maps/rtabmap.db and reused on every start.
OMNIMOW_SLAM_MODE=mapping      keep adding to the map (new sessions are linked by loop closures)
OMNIMOW_SLAM_MODE=localization only localize against the existing map (most stable once mapped)
Always start from the charging station: the map origin is where the first session started.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

STEREO_REMAPS = [
    ('left/image_rect', '/stereo/left/image_rect'),
    ('left/camera_info', '/stereo/left/camera_info'),
    ('right/image_rect', '/stereo/right/image_rect'),
    ('right/camera_info', '/stereo/right/camera_info'),
]


def generate_launch_description():
    localization = os.getenv('OMNIMOW_SLAM_MODE', 'mapping').lower() == 'localization'
    database = os.getenv('OMNIMOW_SLAM_DATABASE', '/maps/rtabmap.db')

    common = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',          # Use FusionCore's fused odom TF
        'approx_sync': False,             # Left/right images carry identical stamps
        'Reg/Force3DoF': 'true',          # Ground robot: x, y, yaw
        'Optimizer/GravitySigma': '0',
    }

    odometry = Node(
        package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry', output='screen',
        parameters=[common, {
            'guess_frame_id': 'odom',     # FusionCore's estimate seeds the visual odometry
            'publish_tf': False,          # FusionCore owns odom -> base_link
            'publish_null_when_lost': False,
            'Odom/ResetCountdown': '1',   # Recover automatically when tracking is lost
            'Vis/MaxDepth': '6.0',        # 6 cm baseline: depth beyond ~6 m is too noisy
        }],
        remappings=STEREO_REMAPS + [('odom', '/rtabmap/stereo_odometry')],
    )

    slam_params = {
        'subscribe_stereo': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'database_path': database,
        'Vis/MaxDepth': '6.0',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/LinearUpdate': '0.1',       # Add a node every 10 cm / 0.1 rad while mowing
        'RGBD/AngularUpdate': '0.1',
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        parameters=[common, slam_params],
        remappings=STEREO_REMAPS,
    )
    print(f"[OmniMow] RTAB-Map in {'localization' if localization else 'mapping'} mode, database {database}")
    return LaunchDescription([odometry, slam])
