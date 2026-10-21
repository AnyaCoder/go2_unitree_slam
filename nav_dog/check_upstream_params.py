#!/usr/bin/env python3
"""检查运行中的规划节点参数是否与上游 autonomy_stack_go2 的 launch 声明一致。

用法（在狗上，先 source 好 ROS 环境、导航已启动）：
    python3 /home/unitree/go2_nav/check_upstream_params.py

只报差异。sensorOffsetX/Y 与 pathFolder 属于必须按本机硬件/路径变化的三项，
单独列出来供确认，不计为错误。
"""
import importlib.util
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'go2_nav_launch', os.path.join(HERE, 'go2_nav.launch.py'))
launch_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_mod)

TARGETS = [
    ('/localPlanner', 'local_planner', 'local_planner.launch', 'localPlanner', None),
    ('/pathFollower', 'local_planner', 'local_planner.launch', 'pathFollower',
     {'sport_request_topic': '/api/sport/request', 'pathFollowerGoalTopic': '/goal_point'}),
    ('/terrainAnalysis', 'terrain_analysis', 'terrain_analysis.launch', 'terrainAnalysis', None),
    ('/terrainAnalysisExt', 'terrain_analysis_ext', 'terrain_analysis_ext.launch',
     'terrainAnalysisExt', {'checkTerrainConn': 'true'}),
]

HARDWARE_KEYS = ('sensorOffsetX', 'sensorOffsetY', 'pathFolder')


def dump(node):
    out = subprocess.run(['ros2', 'param', 'dump', '--print', node],
                         check=True, capture_output=True, text=True).stdout
    data = yaml.safe_load(out)
    if not isinstance(data, dict):
        raise RuntimeError('unexpected dump output: %r' % out[:200])
    body = data[node] if node in data else data[list(data)[0]]
    return body.get('ros__parameters', body)


def main():
    bad = 0
    for node, pkg, launch_file, exe, overrides in TARGETS:
        ref, _ = launch_mod.read_upstream(pkg, launch_file, exe, overrides)
        try:
            live = dump(node)
        except (subprocess.CalledProcessError, FileNotFoundError) as exc:
            print('%s: 取不到参数（节点没在跑？） %s' % (node, exc))
            bad += 1
            continue
        diffs = []
        for key, want in sorted(ref.items()):
            if key in HARDWARE_KEYS or key not in live:
                continue
            got = live[key]
            if isinstance(want, (int, float)) and not isinstance(want, bool):
                same = abs(float(got) - float(want)) < 1e-9
            else:
                same = str(got).lower() == str(want).lower()
            if not same:
                diffs.append((key, got, want))
        print('%s  上游 %d 项  %s' % (node, len(ref), '一致' if not diffs else '有 %d 项不一致' % len(diffs)))
        for key, got, want in diffs:
            print('    %-28s 运行=%r 上游=%r' % (key, got, want))
        for key in HARDWARE_KEYS:
            if key in live:
                print('    (硬件项) %-20s 运行=%r' % (key, live[key]))
        bad += len(diffs)

    print('\n结果:', '全部与上游一致' if bad == 0 else '共 %d 处不一致' % bad)
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
