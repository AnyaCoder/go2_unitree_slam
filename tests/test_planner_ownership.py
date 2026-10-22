#!/usr/bin/env python3
"""Ownership checks for dog-side planning and display-only laptop."""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'nav_dog'))

from far_viz_protocol import TOPICS  # noqa: E402


def read(path):
    return (ROOT / path).read_text(encoding='utf-8')


def active_lines(path):
    return '\n'.join(
        line for line in read(path).splitlines()
        if not line.lstrip().startswith('#'))


def compact(path):
    return re.sub(r'\s+', ' ', active_lines(path))


def squash(path):
    return re.sub(r'\s+', '', active_lines(path))


def test_dog_launch_runs_the_full_upstream_closure():
    launch = compact('nav_dog/go2_nav.launch.py')
    for executable in ('terrainAnalysis', 'terrainAnalysisExt', 'localPlanner',
                       'pathFollower', 'far_planner'):
        assert "executable='%s'" % executable in launch
    assert "name='go2_nav_pose_bridge'" in launch
    assert "name='go2_static_map_replay'" in launch
    assert "'--map', static_map_path" in launch
    assert "package='nav_dog_cpp', executable='cloud_to_map_filter'" in launch
    assert "name='cloud_to_map_filter'" in launch
    assert "sensor_offset_x', default_value='0.16143'" in launch
    assert "max_speed', default_value='0.5'" in launch
    assert "planner_params['goalX'] = float('nan')" in launch
    assert "planner_params['goalY'] = float('nan')" in launch
    assert "follower_params['collisionCloudTimeout'] = 2.0" in launch
    assert 'FAR_CONTOUR_PADDING_M = 0.12' in launch
    assert "'c_detector/contour_padding_m': FAR_CONTOUR_PADDING_M" in launch
    assert "'is_pub_boundary': True" in launch
    assert "('/way_point', '/far/way_point')" not in launch
    assert "('/api/sport/request', sport_out)" in launch


def test_dog_planners_consume_only_the_fused_navigation_state():
    launch = compact('nav_dog/go2_nav.launch.py')
    assert "GLOBAL_ODOM_TOPIC = '/lio_sam_ros2/mapping/re_location_odometry'" in launch
    assert "NAV_STATE_TOPIC = '/nav/state_estimation'" in launch
    assert "return [('/state_estimation', NAV_STATE_TOPIC), " in launch
    assert "('/odom_world', NAV_STATE_TOPIC)" in launch
    assert launch.count('GLOBAL_ODOM_TOPIC') == 2
    assert "'-p', 'input_topic:=' + GLOBAL_ODOM_TOPIC" in launch
    assert 'imu_incremental' not in launch


def test_pose_bridge_does_not_use_imu_preintegration_odometry():
    bridge = read('nav_dog/nav_pose_bridge.py')
    assert 'imu_incremental' not in bridge
    assert "declare_parameter(\n            'input_topic'" in bridge
    assert "input_timeout" in bridge


def test_static_map_replay_is_the_default_planner_cloud_source():
    launch = compact('nav_dog/go2_nav.launch.py')
    cloud = compact('nav_dog_cpp/src/cloud_to_map_filter.cpp')
    assert "RAW_CLOUD_TOPIC = '/lio_sam_ros2/deskew/cloud_deskewed'" in launch
    assert "FILTERED_CLOUD_TOPIC = '/nav/cloud_registered_map'" in launch
    assert "STATIC_CLOUD_TOPIC = '/nav/static_cloud_map'" in launch
    assert "'start_static_replay', default_value='true'" in launch
    assert "'start_cloud_filter', default_value='false'" in launch
    assert "'cloud_topic', default_value=STATIC_CLOUD_TOPIC" in launch
    assert "'--voxel', '0.08'" in launch
    assert "'--window-size', '6.0'" in launch
    assert "'--rate', '5.0'" in launch
    assert "'output_rate_hz': 5.0" in launch
    assert "'input_topic': RAW_CLOUD_TOPIC" in launch
    assert '"output_rate_hz", 5.0' in cloud
    assert '"target_frame", "map"' in cloud
    assert '"input_frame", "base_link"' in cloud
    assert 'unexpected cloud frame' in cloud
    assert 'fromROSMsg' not in cloud
    assert 'toROSMsg' not in cloud
    assert '/lio_sam_ros2/mapping/cloud_registered' not in launch
    assert '/lio_sam_ros2/mapping/cloud_registered' not in cloud


def test_terrain_classification_matches_map_sim():
    launch = compact('nav_dog/go2_nav.launch.py')
    assert 'SIM_TERRAIN_VEHICLE_HEIGHT = 0.45' in launch
    assert "'noDataObstacle': False" in launch
    assert "'vehicleHeight': SIM_TERRAIN_VEHICLE_HEIGHT" in launch
    assert "'scanVoxelSize': 0.05" in launch
    assert "'scanVoxelSize': 0.1" in launch
    assert "'checkTerrainConn': True" in launch
    assert "'localTerrainMapRadius': 4.0" in launch


def test_run_nav_starts_far_and_passes_hardware_limits():
    run_nav = compact('nav_dog/run_nav.sh')
    assert 'start_far:=true' in run_nav
    assert 'max_speed:="$GO2_MAX_SPEED"' in run_nav
    assert 'sensor_offset_x:="$SENSOR_OFFSET_X"' in run_nav


def test_start_script_deploys_new_chain_and_rejects_legacy_owners():
    startup = compact('nav/start_far_real.sh')
    for filename in ('nav_pose_bridge.py', 'static_map_replay.py',
                     'nav_rate_audit.py', 'go2_nav.launch.py'):
        assert filename in startup
    assert 'GlobalMap_clean_20261020_1920_nav_green_edited.pcd' in startup
    assert 'GO2_FORCE_STATIC_MAP_SYNC' in startup
    assert 'remote_size' in startup
    assert '复用狗端固定导航 PCD' in startup
    for label in ('nav_pose_bridge', 'static_map_replay', 'terrainAnalysis',
                  'terrainAnalysisExt', 'far_planner', 'localPlanner',
                  'pathFollower'):
        assert label in startup
    assert '[c]loud_to_map_filter' not in startup
    assert '[g]lobal_path_receiver.py' in startup
    assert '[d]og_control_B1_one' in startup
    assert '[g]o2_control_by_sdk.*send_cmd' in startup


def test_goalpoint_replaces_global_path_transport():
    view = active_lines('nav/view_real.sh')
    assert 'nav/goal_sender.py' in view
    assert 'run_far_command_link.sh' not in view
    assert 'run_global_path_receiver.sh' not in view
    assert 'nav/global_path_sender.py' not in view


def test_laptop_view_is_display_only():
    view = active_lines('nav/view_real.sh')
    assert 'nav/far_viz_receiver.py' in view
    assert '--map "$MAP"' in view
    assert 'robot_state_publisher' in view
    assert 'rviz2 -d "$RVIZ_CONFIG"' in view
    for forbidden in (
        r'\bssh\b',
        r'\bscp\b',
        r'\bpkill\b',
        r'kill\s+-TERM',
        r'nav_pose_bridge',
        r'static_map_replay',
        r'far_planner',
        r'localPlanner',
        r'pathFollower',
        r'ros2\s+launch\s+map_sim\s+map_sim\.launch\.py',
        r'ros2\s+launch\s+.*global_planner\.launch\.py',
    ):
        assert re.search(forbidden, view) is None, forbidden


def test_view_processes_are_reused_without_stopping_services():
    view = active_lines('nav/view_real.sh')
    stop = active_lines('nav/stop_view.sh')
    assert 'RUNTIME_DIR' in view and '.pid' in view
    assert '复用 $name' in view
    assert '接管已有 $name' in view
    assert 'nohup' in view
    assert 'RUNTIME_DIR' in stop
    assert '.pid' in stop
    assert re.search(r'\b(ssh|scp|pkill)\b', stop) is None


def test_view_fails_fast_when_dog_nav_link_is_missing():
    view = active_lines('nav/view_real.sh')
    startup = active_lines('nav/start_far_real.sh')
    assert 'tcp_open "$HOST_IP" "$STATE_PORT"' in view
    assert '狗端导航链未启动' in view
    assert './nav/start_far_real.sh' in view
    assert '关键进程被挂起' in startup


def test_uplink_contains_state_and_native_far_visualization_only():
    topics = {spec.remote_topic for spec in TOPICS}
    required = {
        '/nav/state_estimation', '/tf', '/tf_static', '/joint_states',
        '/path', '/way_point', '/navigation_boundary',
        '/viz_path_topic', '/viz_graph_topic', '/viz_node_topic',
        '/viz_poly_topic', '/viz_contour_topic', '/viz_grid_map_topic',
        '/viz_viewpoint_extend_topic', '/far_robot_model',
    }
    assert topics == required
    state = next(spec for spec in TOPICS
                 if spec.remote_topic == '/nav/state_estimation')
    assert state.local_topic == '/state_estimation'


def test_no_live_cloud_or_motion_command_is_transported():
    protocol = read('nav_dog/far_viz_protocol.py')
    link = read('nav_dog/far_viz_link.py')
    receiver = read('nav/far_viz_receiver.py')
    for source in (protocol, link, receiver):
        assert 'OP_CLOUD' not in source
        assert '/go2/min_obstacle_cloud' not in source
        assert '/added_obstacles' not in source
    assert '/lio_sam_ros2/mapping/lasercloud_in_world' not in link
    topics = {spec.remote_topic for spec in TOPICS}
    for forbidden in ('/registered_scan', '/terrain_map', '/terrain_map_ext',
                      '/nav/cloud_registered_map', '/nav/static_cloud_map',
                      '/api/sport/request',
                      '/goal_point', '/cmd_vel'):
        assert forbidden not in topics


def test_fixed_display_map_is_latched_once():
    receiver = read('nav/far_viz_receiver.py')
    rviz = read('nav/far_real.rviz')
    assert "PointCloud2, '/overall_map', static_qos" in receiver
    assert 'self.publish_map()' in receiver
    assert 'create_timer(1.0, self.publish_map)' not in receiver
    assert 'Value: /overall_map' in rviz
    assert 'Durability Policy: Transient Local' in rviz


def test_rviz_has_no_live_cloud_layers_and_uses_map_frame():
    import yaml
    for path in ('nav/map_sim.rviz', 'nav/far_real.rviz'):
        with (ROOT / path).open(encoding='utf-8') as stream:
            manager = yaml.safe_load(stream)['Visualization Manager']
        assert manager['Global Options']['Fixed Frame'] == 'map'
        assert manager['Tools']
    rviz = read('nav/far_real.rviz')
    for name in ('RegScan', 'TerrainMap', 'TerrainMapExt',
                 'Local Collision Obstacles', 'Go2 Live Cloud'):
        assert 'Name: %s\n' % name not in rviz


def test_map_sim_keeps_the_known_working_upstream_closure():
    sim = active_lines('nav/run_map_sim.sh')
    assert 'ros2 launch map_sim map_sim.launch.py' in sim
    assert 'space_viz:=true' in sim
    assert 'nav/static_map_publisher.py' in sim
    assert 'nav/map_sim.rviz' in sim
    assert '人工可走区: enabled' in sim
