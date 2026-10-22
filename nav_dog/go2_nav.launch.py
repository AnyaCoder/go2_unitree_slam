#!/usr/bin/env python3
"""Go2 full navigation stack, using the installed upstream ROS 2 launch files.

The Go2 owns the complete planning and execution loop:

    LIO IMU odom + global correction -> nav_pose_bridge -> 50 Hz map pose
    fixed navigation PCD -> static_map_replay -> 5 Hz rolling map terrain
                    -> terrainAnalysis -> terrainAnalysisExt
                    -> FAR -> /way_point -> localPlanner -> /path
                    -> pathFollower -> /api/sport/request

The laptop is display-only. It receives low-rate state and FAR visualization
messages over TCP 8898 and sends only the operator's final /goal_point over TCP
8899. UnitreeSlam legacy motion controllers are disabled outside this launch.

The only deliberate upstream adaptations are:
  * LIO-SAM supplies pose and relocalization; it is not a terrain input.
  * nav_pose_bridge propagates 3 Hz global localization at 50 Hz using the
    existing IMU preintegration odometry.
  * static_map_replay emits fixed-PCD rolling terrain: 0.08 m voxel,
    6 m x 6 m window, 5 Hz map-frame XYZI clouds.
  * Terrain classification uses map_sim's exact values, not the upstream
    real-robot defaults.
  * MID-360 sensorOffsetX/Y replace the upstream L1 mounting defaults.
  * localPlanner starts with NaN goals so relocalization cannot trigger motion.
  * pathFollower tolerates the real robot's approximately 1 Hz collision cloud.
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HIGH_RATE_ODOM_TOPIC = '/odometry/imu_incremental'
GLOBAL_ODOM_TOPIC = '/lio_sam_ros2/mapping/re_location_odometry'
NAV_STATE_TOPIC = '/nav/state_estimation'
RAW_CLOUD_TOPIC = '/lio_sam_ros2/deskew/cloud_deskewed'
FILTERED_CLOUD_TOPIC = '/nav/cloud_registered_map'
STATIC_CLOUD_TOPIC = '/nav/static_cloud_map'
DEFAULT_STATIC_MAP = (
    '/home/unitree/go2_nav/maps/'
    'GlobalMap_clean_20261020_1920_nav_green_edited.pcd')
FAR_CONTOUR_PADDING_M = 0.12
SIM_TERRAIN_VEHICLE_HEIGHT = 0.45


def _subst(text, args):
    """Expand $(var name) and $(find-pkg-share pkg) in upstream launch XML."""
    if text is None:
        return None
    out = text
    while True:
        start = out.find('$(var ')
        if start < 0:
            break
        end = out.index(')', start)
        name = out[start + 6:end].strip()
        value = args[name]
        if start == 0 and end == len(out) - 1:
            return value
        out = out[:start] + str(value) + out[end + 1:]
    while '$(find-pkg-share ' in out:
        start = out.index('$(find-pkg-share ')
        end = out.index(')', start)
        package = out[start + 17:end].strip()
        out = out[:start] + get_package_share_directory(package) + out[end + 1:]
    return out


def _typed(value):
    if not isinstance(value, str):
        return value
    if value.strip().lower() in ('true', 'false'):
        return value.strip().lower() == 'true'
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def read_upstream(pkg, launch_name, exec_name, override_args=None):
    """Return one upstream node's parameters and remappings."""
    path = get_package_share_directory(pkg) + '/launch/' + launch_name
    root = ET.parse(path).getroot()
    args = {item.get('name'): item.get('default', '')
            for item in root.findall('arg')}
    if override_args:
        args.update(override_args)

    params, remaps = {}, []
    for node in root.findall('node'):
        if node.get('exec') != exec_name:
            continue
        for param in node.findall('param'):
            if param.get('from'):
                continue
            params[param.get('name')] = _typed(_subst(param.get('value'), args))
        for remap in node.findall('remap'):
            remaps.append((_subst(remap.get('from'), args),
                           _subst(remap.get('to'), args)))
    if not params:
        raise RuntimeError('%s has no node %s' % (path, exec_name))
    return params, remaps


def lio_remaps(cloud_topic, *extra):
    """Connect upstream terrain/local nodes to the fused map-frame state."""
    return [('/state_estimation', NAV_STATE_TOPIC),
            ('/registered_scan', cloud_topic)] + list(extra)


def generate_launch_description():
    sport_out = LaunchConfiguration('sport_out')
    sensor_x = LaunchConfiguration('sensor_offset_x')
    sensor_y = LaunchConfiguration('sensor_offset_y')
    max_speed = LaunchConfiguration('max_speed')
    start_far = LaunchConfiguration('start_far')
    start_cloud_filter = LaunchConfiguration('start_cloud_filter')
    start_pose_bridge = LaunchConfiguration('start_pose_bridge')
    start_static_replay = LaunchConfiguration('start_static_replay')
    static_map_path = LaunchConfiguration('static_map_path')
    cloud_topic = LaunchConfiguration('cloud_topic')

    planner_params, _ = read_upstream(
        'local_planner', 'local_planner.launch', 'localPlanner',
        {'autonomyMode': 'true', 'maxSpeed': max_speed})
    # Upstream autonomyMode defaults to (0, 0). A fresh relocalization must not
    # produce a path toward map origin before the operator sends a Goalpoint.
    planner_params['goalX'] = float('nan')
    planner_params['goalY'] = float('nan')

    follower_params, follower_remaps = read_upstream(
        'local_planner', 'local_planner.launch', 'pathFollower',
        {'autonomyMode': 'true', 'maxSpeed': max_speed})
    follower_params['collisionCloudTimeout'] = 2.0
    follower_remaps += [('/api/sport/request', sport_out)]

    # Keep this block byte-for-byte equivalent in meaning to map_sim's launch.
    # The upstream real-robot launch uses vehicleHeight=1.5 and
    # noDataObstacle=true, which produces a different terrain than the accepted
    # simulation.
    terrain_params = {
        'scanVoxelSize': 0.05, 'decayTime': 2.0, 'noDecayDis': 0.0,
        'clearingDis': 8.0, 'useSorting': True, 'quantileZ': 0.25,
        'considerDrop': False, 'limitGroundLift': False,
        'maxGroundLift': 0.15, 'clearDyObs': False,
        'minDyObsDis': 0.3, 'minDyObsAngle': 0.0, 'minDyObsRelZ': -0.3,
        'absDyObsRelZThre': 0.2, 'minDyObsVFOV': -16.0,
        'maxDyObsVFOV': 16.0, 'minDyObsPointNum': 1,
        'noDataObstacle': False, 'noDataBlockSkipNum': 0,
        'minBlockPointNum': 10, 'maxElevBelowVeh': -0.6,
        'noDataAreaMinX': 0.3, 'noDataAreaMaxX': 1.8,
        'noDataAreaMinY': -0.9, 'noDataAreaMaxY': 0.9,
        'vehicleHeight': SIM_TERRAIN_VEHICLE_HEIGHT,
        'voxelPointUpdateThre': 100, 'voxelTimeUpdateThre': 2.0,
        'minRelZ': -1.5, 'maxRelZ': 0.5, 'disRatioZ': 0.2,
    }
    terrain_ext_params = {
        'scanVoxelSize': 0.1, 'decayTime': 10.0, 'noDecayDis': 0.0,
        'clearingDis': 30.0, 'useSorting': True, 'quantileZ': 0.1,
        'vehicleHeight': SIM_TERRAIN_VEHICLE_HEIGHT,
        'voxelPointUpdateThre': 100, 'voxelTimeUpdateThre': 2.0,
        'lowerBoundZ': -2.5, 'upperBoundZ': 1.0, 'disRatioZ': 0.1,
        'checkTerrainConn': True, 'terrainConnThre': 0.5,
        'terrainUnderVehicle': -0.75, 'ceilingFilteringThre': 2.0,
        'localTerrainMapRadius': 4.0,
    }

    sensor_params = {'sensorOffsetX': sensor_x, 'sensorOffsetY': sensor_y}
    script_dir = os.path.dirname(__file__)
    pose_bridge_script = os.path.join(script_dir, 'nav_pose_bridge.py')
    static_map_script = os.path.join(script_dir, 'static_map_replay.py')
    pose_bridge = ExecuteProcess(
        cmd=[
            'python3', pose_bridge_script, '--ros-args',
            '-p', 'high_rate_topic:=' + HIGH_RATE_ODOM_TOPIC,
            '-p', 'global_topic:=' + GLOBAL_ODOM_TOPIC,
            '-p', 'output_topic:=' + NAV_STATE_TOPIC,
        ],
        name='go2_nav_pose_bridge',
        output='screen',
        condition=IfCondition(start_pose_bridge),
    )
    static_replay = ExecuteProcess(
        cmd=[
            'python3', static_map_script,
            '--map', static_map_path,
            '--pose-topic', NAV_STATE_TOPIC,
            '--output-topic', STATIC_CLOUD_TOPIC,
            '--frame', 'map',
            '--voxel', '0.08',
            '--window-size', '6.0',
            '--rate', '5.0',
            '--pose-timeout', '1.0',
        ],
        name='go2_static_map_replay',
        output='screen',
        condition=IfCondition(start_static_replay),
    )
    cloud_filter = Node(
        package='nav_dog_cpp', executable='cloud_to_map_filter',
        name='cloud_to_map_filter', output='screen',
        parameters=[{
            'input_topic': RAW_CLOUD_TOPIC,
            'output_topic': FILTERED_CLOUD_TOPIC,
            'pose_topic': NAV_STATE_TOPIC,
            'input_frame': 'base_link',
            'target_frame': 'map',
            'output_rate_hz': 5.0,
            'self_radius': 0.70,
            'self_z_band': 0.65,
            'voxel_size': 0.08,
            'pose_timeout': 0.40,
        }],
        condition=IfCondition(start_cloud_filter),
    )

    terrain = Node(
        package='terrain_analysis', executable='terrainAnalysis',
        name='terrainAnalysis', output='screen',
        parameters=[terrain_params], remappings=lio_remaps(cloud_topic))
    terrain_ext = Node(
        package='terrain_analysis_ext', executable='terrainAnalysisExt',
        name='terrainAnalysisExt', output='screen',
        parameters=[terrain_ext_params], remappings=lio_remaps(cloud_topic))
    planner = Node(
        package='local_planner', executable='localPlanner',
        name='localPlanner', output='screen',
        parameters=[planner_params, sensor_params],
        remappings=lio_remaps(cloud_topic))
    follower = Node(
        package='local_planner', executable='pathFollower',
        name='pathFollower', output='screen',
        parameters=[follower_params, sensor_params],
        remappings=lio_remaps(cloud_topic, *follower_remaps))

    far = Node(
        package='far_planner', executable='far_planner', name='far_planner',
        output='screen', condition=IfCondition(start_far),
        parameters=[
            os.path.join(
                get_package_share_directory('far_planner'),
                'config', 'default.yaml'),
            {
                'c_detector/contour_padding_m': FAR_CONTOUR_PADDING_M,
                'is_pub_boundary': True,
            },
        ],
        # FAR publishes /way_point directly. Do not introduce a second global
        # path receiver or remap this topic away from localPlanner.
        remappings=[
            ('/odom_world', NAV_STATE_TOPIC),
            ('/terrain_cloud', '/terrain_map_ext'),
            ('/scan_cloud', '/terrain_map'),
            ('/terrain_local_cloud', cloud_topic),
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'sport_out', default_value='/api/sport/request',
            description='运动指令话题；dry run 用 /nav/dry_cmd'),
        DeclareLaunchArgument(
            'sensor_offset_x', default_value='0.16143',
            description='Livox MID-360 相对车体中心的 x 偏移'),
        DeclareLaunchArgument(
            'sensor_offset_y', default_value='0.0',
            description='Livox MID-360 相对车体中心的 y 偏移'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.5',
            description='真机线速度上限（m/s）'),
        DeclareLaunchArgument(
            'start_pose_bridge', default_value='true',
            description='把高频 IMU odom 与低频全局定位融合为 50 Hz 导航状态'),
        DeclareLaunchArgument(
            'start_static_replay', default_value='true',
            description='用固定导航 PCD 生成 6 m x 6 m / 5 Hz 滚动点云'),
        DeclareLaunchArgument(
            'static_map_path', default_value=DEFAULT_STATIC_MAP,
            description='狗端固定导航 PCD'),
        DeclareLaunchArgument(
            'start_cloud_filter', default_value='false',
            description='可选旧模式：把 10 Hz deskew cloud 转到 map 并降到 5 Hz'),
        DeclareLaunchArgument(
            'cloud_topic', default_value=STATIC_CLOUD_TOPIC,
            description='terrain/localPlanner 消费的 map-frame 点云'),
        DeclareLaunchArgument(
            'start_far', default_value='true',
            description='狗端启动完整 FAR 规划链'),
        pose_bridge, static_replay, cloud_filter, terrain, terrain_ext,
        planner, follower, far,
    ])
