#!/usr/bin/env python3
"""Go2 点选导航：参数直接取自 autonomy_stack_go2 上游 launch，不另立一套。

做法：运行时解析上游 launch XML（local_planner.launch / terrain_analysis.launch /
terrain_analysis_ext.launch）里的 <arg>/<param>/<remap>，原样传给节点。
这里的数值就等于上游默认值（含 $(var ...) 展开），上游改了参数我们也自动跟随，
不会出现"手抄一份参数"慢慢漂移的问题。

相对上游 system_real_robot_with_route_planner.launch，本文件只做三件事：
  1. 用 LIO-SAM 替换 Point-LIO：/state_estimation、/registered_scan 重映射到
     /lio_sam_ros2/mapping/{re_location_odometry,cloud_registered}；
     far_planner 的 /odom_world 直接接真实里程计，避免 ROS2 多级 remap 不串联
  2. 暴露 sensorOffsetX/Y：唯一真正取决于硬件安装位置的量。上游默认 0.3/0.0
     对应 L1 装在狗头前方 0.3 m；MID360 装在背上时要用实测值
  3. 先把 cloud_registered 从 odom 转到 map 并过滤狗体回波，再交给
     terrainAnalysis/localPlanner；TF 不可用时宁可丢帧也不转发错误坐标
  4. 不启动任何控制桥；状态上行、waypoint 和初始位姿分别由
     run_real_view.sh 管理的独立 Python 进程负责

不启动（与上游一致或本机不需要）：joy_node（狗上没有 /dev/input/js0）、
point_lio_unilidar（已由 LIO-SAM 取代）、rviz2、
go2_camera_bridge（相机桥由其它服务占用）、/sensor -> /vehicle 静态 TF
（规划链只看 /state_estimation，不看 TF）。
"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODOM_TOPIC = '/lio_sam_ros2/mapping/re_location_odometry'
CLOUD_TOPIC = '/lio_sam_ros2/mapping/cloud_registered'
FILTERED_CLOUD_TOPIC = '/nav/cloud_registered_map'


def _subst(text, args):
    """展开 $(var name) / $(find-pkg-share pkg)。

    若整段文本就是一个 $(var x)，原样返回 args 里的对象，
    这样 LaunchConfiguration 能一路传到 remap / param 里。
    """
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
            return value                      # 保持 LaunchConfiguration 类型
        out = out[:start] + str(value) + out[end + 1:]
    while '$(find-pkg-share ' in out:
        start = out.index('$(find-pkg-share ')
        end = out.index(')', start)
        pkg = out[start + 17:end].strip()
        out = out[:start] + get_package_share_directory(pkg) + out[end + 1:]
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
    """返回上游同名节点的 (params, remappings)。"""
    path = get_package_share_directory(pkg) + '/launch/' + launch_name
    root = ET.parse(path).getroot()

    args = {a.get('name'): a.get('default', '') for a in root.findall('arg')}
    if override_args:
        args.update(override_args)

    params, remaps = {}, []
    for node in root.findall('node'):
        if node.get('exec') != exec_name:
            continue
        for param in node.findall('param'):
            if param.get('from'):             # 外部 yaml 由调用方处理
                continue
            params[param.get('name')] = _typed(_subst(param.get('value'), args))
        for remap in node.findall('remap'):
            remaps.append((_subst(remap.get('from'), args),
                           _subst(remap.get('to'), args)))
    if not params:
        raise RuntimeError('%s 里没找到节点 %s 的参数' % (path, exec_name))
    return params, remaps


def lio_remaps(cloud_topic, *extra):
    """把上游输入接到 LIO-SAM 位姿和已校验的 map-frame 点云。"""
    return [('/state_estimation', ODOM_TOPIC),
            ('/registered_scan', cloud_topic)] + list(extra)


def generate_launch_description():
    sport_out = LaunchConfiguration('sport_out')
    sensor_x = LaunchConfiguration('sensor_offset_x')
    sensor_y = LaunchConfiguration('sensor_offset_y')
    start_far = LaunchConfiguration('start_far')
    start_cloud_filter = LaunchConfiguration('start_cloud_filter')
    cloud_topic = LaunchConfiguration('cloud_topic')

    planner_params, _ = read_upstream(
        'local_planner', 'local_planner.launch', 'localPlanner',
        {'autonomyMode': 'true'})
    follower_params, follower_remaps = read_upstream(
        'local_planner', 'local_planner.launch', 'pathFollower',
        {'sport_request_topic': sport_out, 'pathFollowerGoalTopic': '/goal_point',
         'autonomyMode': 'true'})
    # LIO-SAM's registered cloud currently arrives at about 1 Hz on the real
    # robot.  The upstream 0.5 s timeout makes pathFollower pause almost every
    # other frame; 2 s tolerates one dropped frame but still stops if stale.
    follower_params['collisionCloudTimeout'] = 2.0
    terrain_params, _ = read_upstream(
        'terrain_analysis', 'terrain_analysis.launch', 'terrainAnalysis')
    terrain_ext_params, _ = read_upstream(
        'terrain_analysis_ext', 'terrain_analysis_ext.launch', 'terrainAnalysisExt',
        {'checkTerrainConn': 'true'})         # 上游 system_real_robot 传的就是 true

    sensor_params = {'sensorOffsetX': sensor_x, 'sensorOffsetY': sensor_y}
    self_filter_script = os.path.join(os.path.dirname(__file__), 'self_filter.py')
    cloud_filter = ExecuteProcess(
        cmd=['python3', self_filter_script],
        name='go2_cloud_frame_self_filter', output='screen',
        condition=IfCondition(start_cloud_filter))

    terrain = Node(package='terrain_analysis', executable='terrainAnalysis',
                   name='terrainAnalysis', output='screen',
                   parameters=[terrain_params], remappings=lio_remaps(cloud_topic))

    terrain_ext = Node(package='terrain_analysis_ext', executable='terrainAnalysisExt',
                       name='terrainAnalysisExt', output='screen',
                       parameters=[terrain_ext_params], remappings=lio_remaps(cloud_topic))

    planner = Node(package='local_planner', executable='localPlanner',
                   name='localPlanner', output='screen',
                   parameters=[planner_params, sensor_params],
                   remappings=lio_remaps(cloud_topic))

    follower = Node(package='local_planner', executable='pathFollower',
                    name='pathFollower', output='screen',
                    parameters=[follower_params, sensor_params],
                    remappings=lio_remaps(cloud_topic, *follower_remaps))

    far = Node(package='far_planner', executable='far_planner', name='far_planner',
               output='screen',
               condition=IfCondition(start_far),
               parameters=[get_package_share_directory('far_planner') + '/config/default.yaml'],
               remappings=[('/odom_world', ODOM_TOPIC),
                           ('/terrain_cloud', '/terrain_map_ext'),
                           ('/scan_cloud', '/terrain_map'),
                           ('/terrain_local_cloud', cloud_topic)])

    return LaunchDescription([
        DeclareLaunchArgument('sport_out', default_value='/api/sport/request',
                              description='运动指令话题；dry run 用 /nav/dry_cmd'),
        DeclareLaunchArgument('sensor_offset_x', default_value='0.3',
                              description='上游默认 0.3（L1 在狗头前 0.3m）；MID360 装背上需实测'),
        DeclareLaunchArgument('sensor_offset_y', default_value='0.0'),
        DeclareLaunchArgument(
            'start_cloud_filter', default_value='true',
            description='把 cloud_registered 从 odom 转到 map 并删除自身回波'),
        DeclareLaunchArgument(
            'cloud_topic', default_value=FILTERED_CLOUD_TOPIC,
            description='terrain/localPlanner 消费的 map-frame 点云'),
        DeclareLaunchArgument(
            'start_far', default_value='false',
            description='电脑端已负责静态地图 FAR；狗端默认不重复启动 FAR'),
        cloud_filter, terrain, terrain_ext, planner, follower, far,
    ])
