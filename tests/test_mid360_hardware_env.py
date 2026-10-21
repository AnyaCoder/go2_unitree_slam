#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENV_FILE = ROOT / 'nav_dog' / 'go2_mid360_hardware.env'
KEYS = (
    'SENSOR_OFFSET_X', 'SENSOR_OFFSET_Y',
    'FAR_SENSOR_OFFSET_X', 'FAR_SENSOR_OFFSET_Y',
    'GO2_MID360_SENSOR_OFFSET_X', 'GO2_MID360_SENSOR_OFFSET_Y',
    'GO2_MAX_SPEED',
)


def read_offsets(extra):
    env = os.environ.copy()
    for key in KEYS:
        env.pop(key, None)
    env.update(extra)
    script = (
        f'source {str(ENV_FILE)!r}\n'
        'printf "%s %s %s %s %s\\n" '
        '"$SENSOR_OFFSET_X" "$SENSOR_OFFSET_Y" '
        '"$FAR_SENSOR_OFFSET_X" "$FAR_SENSOR_OFFSET_Y" '
        '"$GO2_MAX_SPEED"\n'
    )
    result = subprocess.run(
        ['bash', '-c', script], env=env, check=True,
        text=True, capture_output=True)
    return [float(value) for value in result.stdout.split()]


def test_mid360_defaults_are_shared():
    assert read_offsets({}) == [
        0.16143, 0.0, 0.16143, 0.0, 0.5]


def test_sensor_override_reaches_waypoint_bridge():
    assert read_offsets({'SENSOR_OFFSET_X': '0.187'}) == [
        0.187, 0.0, 0.187, 0.0, 0.5]


def test_explicit_bridge_override_is_preserved():
    assert read_offsets({
        'SENSOR_OFFSET_X': '0.187',
        'FAR_SENSOR_OFFSET_X': '0.170',
    }) == [0.187, 0.0, 0.170, 0.0, 0.5]


def test_real_speed_default_and_override():
    assert read_offsets({})[-1] == 0.5
    assert read_offsets({'GO2_MAX_SPEED': '0.25'})[-1] == 0.25


if __name__ == '__main__':
    test_mid360_defaults_are_shared()
    test_sensor_override_reaches_waypoint_bridge()
    test_explicit_bridge_override_is_preserved()
    test_real_speed_default_and_override()
    print('MID360 hardware env tests passed')
