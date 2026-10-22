# nav_dog_cpp

Native C++17 implementation of the Go2 map-cloud adapter. It replaces the
Python/NumPy `self_filter` hot path without changing FAR, localPlanner, or
pathFollower behavior.

## Runtime contract

```text
input:
  /lio_sam_ros2/deskew/cloud_deskewed  sensor_msgs/PointCloud2, base_link
pose:
  /nav/state_estimation                nav_msgs/Odometry, map -> base_link
output:
  /nav/cloud_registered_map            sensor_msgs/PointCloud2, map, <= 5 Hz
```

The filter fails closed when the frame IDs are wrong, the pose is stale, the
point layout is invalid, or the cloud is big-endian. It removes points inside a
0.70 m radius and +/-0.65 m vertical band around the robot, then deduplicates on
an 0.08 m voxel grid.

## Hot-path design

- Direct `PointCloud2` field-offset parsing; no PCL message conversion.
- Native-endian scalar reads through `memcpy`, safe for unaligned fields.
- Compact 16-byte `PointXYZI`; output bytes are copied from contiguous storage.
- SplitMix64 hash and preallocated open addressing replace per-point node
  allocation.
- Signed 21-bit voxel keys packed into one 64-bit integer.
- Buffers and the voxel table are cleared and reused across frames.
- Rate limiting occurs before parsing/transformation work.

The default output is `x/y/z/intensity` as four `FLOAT32` fields with
`point_step=16`.

## Build and test

```bash
source /opt/ros/foxy/setup.bash
colcon build --base-paths nav_dog_cpp --packages-select nav_dog_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --base-paths nav_dog_cpp --packages-select nav_dog_cpp
colcon test-result --verbose
```

This package is optional. The accepted real-robot path uses
`static_map_replay` and does not start `cloud_to_map_filter` by default. To test
it manually on the Go2, copy this directory to
`/home/unitree/go2_nav_ws/src/nav_dog_cpp`, build it with the command above,
and launch with `start_cloud_filter:=true` and `start_static_replay:=false`.

## Measured on the Go2

With 13,186 points per cloud and 9.8 Hz accepted input, the node produced the
configured 5 Hz output at 1.39 ms mean and 1.88 ms maximum processing time per
frame, using about 0.7% of one CPU core in the isolated benchmark.
