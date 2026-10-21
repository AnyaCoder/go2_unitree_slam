#!/usr/bin/env python3
"""Laptop global-planning stack on the fixed PCD map.

Reuse the installed upstream ``map_sim.launch.py`` definitions so terrain and
FAR parameters stay pinned to autonomy_stack_go2. Keep only pcdMapSim,
terrainAnalysis, terrainAnalysisExt and far_planner; localPlanner/pathFollower
belong to the Go2 and must never start on the laptop.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

ALLOWED_EXECUTABLES = {
    'pcd_map_sim',
    'terrainAnalysis',
    'terrainAnalysisExt',
}

# FAR's public waypoint is an implementation detail of whichever machine runs
# FAR. Keep the laptop copy off /way_point; the Go2 receiver owns that topic.
FAR_CONTOUR_PADDING_M = 0.12
FAR_REMAPPINGS = [
    ('/odom_world', '/state_estimation'),
    ('/terrain_cloud', '/terrain_map_ext'),
    ('/scan_cloud', '/terrain_map'),
    ('/terrain_local_cloud', '/registered_scan'),
    ('/way_point', '/far/way_point'),
]


def _load_upstream_launch():
    path = os.path.join(
        get_package_share_directory('map_sim'), 'launch', 'map_sim.launch.py')
    spec = importlib.util.spec_from_file_location(
        'go2_fixed_map_upstream_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _entity_executable(entity):
    if not isinstance(entity, Node):
        return None
    return entity.__dict__.get('_Node__node_executable')


def generate_launch_description():
    upstream = _load_upstream_launch()
    entities = []
    for entity in upstream.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entities.append(entity)
        elif _entity_executable(entity) in ALLOWED_EXECUTABLES:
            entities.append(entity)
    found = {_entity_executable(entity) for entity in entities}
    missing = ALLOWED_EXECUTABLES - found
    if missing:
        raise RuntimeError(
            'upstream map_sim launch is missing nodes: %s' %
            ', '.join(sorted(missing)))

    entities.append(Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('far_planner'),
                'config', 'default.yaml'),
            {'c_detector/contour_padding_m': FAR_CONTOUR_PADDING_M},
        ],
        remappings=FAR_REMAPPINGS,
    ))
    return LaunchDescription(entities)
