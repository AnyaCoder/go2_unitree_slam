#!/usr/bin/env python3
"""Static ownership checks for laptop-global / Go2-local planning."""

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


def test_laptop_launch_keeps_only_global_planner_nodes():
    launch = read('nav/global_planner.launch.py')
    assert "'pcd_map_sim'" in launch
    assert "'terrainAnalysis'" in launch
    assert "'terrainAnalysisExt'" in launch
    assert "'far_planner'" in launch
    assert "('/way_point', '/far/way_point')" in launch
    check = read('nav/check_real_chain.sh')
    assert 'require_no_publisher /way_point' in check
    assert '/far/way_point' in check
    assert 'localPlanner' not in launch.split('ALLOWED_EXECUTABLES', 1)[1].split('}', 1)[0]
    assert 'pathFollower' not in launch.split('ALLOWED_EXECUTABLES', 1)[1].split('}', 1)[0]
    assert '/viz_path_topic' not in launch


def test_dog_launch_defaults_far_off_and_has_no_origin_goal():
    launch = read('nav_dog/go2_nav.launch.py')
    run_nav = read('nav_dog/run_nav.sh')
    assert "planner_params['goalX'] = float('nan')" in launch
    assert "planner_params['goalY'] = float('nan')" in launch
    assert "'start_far', default_value='false'" in launch
    assert 'start_far:=false' in run_nav


def test_global_path_channel_replaces_final_goal_channel():
    startup = active_lines('nav/run_real_view.sh')
    assert 'nav/global_path_sender.py' in startup
    assert 'run_global_path_receiver.sh' in startup
    assert 'run_far_command_link.sh' not in startup
    assert 'python3 "${ROOT}/nav/waypoint_sender.py"' not in startup


def test_global_path_sender_is_laptop_side_and_uses_far_marker():
    sender = read('nav/global_path_sender.py')
    assert "Marker, '/viz_path_topic'" in sender
    assert 'pack_global_path_frame' in sender
    assert "PointStamped, '/way_point'" not in sender
    assert "Twist" not in sender


def test_global_path_receiver_is_sole_active_waypoint_owner():
    receiver = read('nav_dog/global_path_receiver.py')
    assert "create_publisher(PointStamped, '/way_point'" in receiver
    assert "create_publisher(PointStamped, '/goal_point'" not in receiver
    assert 'path_stale_seconds' in receiver
    assert 'choose_path_target' in receiver
    dog_launcher = read('nav_dog/run_global_path_receiver.sh')
    assert 'global_path_receiver.py' in dog_launcher


def test_uplink_contains_state_and_local_path_only():
    topics = {spec.remote_topic for spec in TOPICS}
    assert topics == {
        '/lio_sam_ros2/mapping/re_location_odometry',
        '/tf', '/tf_static', '/joint_states', '/path',
    }
    assert '/way_point' not in topics
    assert '/viz_path_topic' not in topics
    assert '/viz_graph_topic' not in topics


def test_laptop_view_runs_global_planner_without_local_execution():
    view = active_lines('nav/run_real_view.sh')
    assert 'nav/global_planner.launch.py' in view
    assert 'external_odom_topic:=/state_estimation' in view
    forbidden = (
        r'^ros2\s+launch\s+map_sim\s+map_sim\.launch\.py',
        r'^/home/.+/(?:localPlanner|pathFollower)',
    )
    for pattern in forbidden:
        assert re.search(pattern, view, re.MULTILINE) is None, pattern


if __name__ == '__main__':
    test_laptop_launch_keeps_only_global_planner_nodes()
    test_dog_launch_defaults_far_off_and_has_no_origin_goal()
    test_global_path_channel_replaces_final_goal_channel()
    test_global_path_sender_is_laptop_side_and_uses_far_marker()
    test_global_path_receiver_is_sole_active_waypoint_owner()
    test_uplink_contains_state_and_local_path_only()
    test_laptop_view_runs_global_planner_without_local_execution()
    print('planner ownership tests passed')
