#!/usr/bin/env python3
"""Read-only Go2 joint/IMU bridge; native DDS and ROS run in separate processes."""
import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import urlopen

JOINTS = [leg + '_' + part + '_joint' for leg in ('FR', 'FL', 'RR', 'RL')
          for part in ('hip', 'thigh', 'calf')]


def robot(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    ChannelFactoryInitialize(0, args.interface)
    lock = threading.Lock()
    state = {"packet": None, "received": 0.0, "sequence": 0}
    sample_period = 1.0 / args.rate
    next_sample = 0.0

    def receive(msg):
        nonlocal next_sample
        now = time.monotonic()
        if now < next_sample:
            return
        next_sample = now + sample_period
        packet = {"position": [float(m.q) for m in msg.motor_state[:12]],
                  "velocity": [float(m.dq) for m in msg.motor_state[:12]],
                  "effort": [float(m.tau_est) for m in msg.motor_state[:12]],
                  "quaternion_wxyz": list(msg.imu_state.quaternion),
                  "gyroscope": list(msg.imu_state.gyroscope),
                  "accelerometer": list(msg.imu_state.accelerometer)}
        if not all(math.isfinite(x) for values in packet.values() for x in values):
            return
        with lock:
            state["sequence"] += 1
            packet["sequence"] = state["sequence"]
            state.update(packet=packet, received=time.monotonic())

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(receive, 1)

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != '/state.json':
                self.send_error(404)
                return
            with lock:
                packet, age = state['packet'], time.monotonic() - state['received']
            if packet is None or age > 1.0:
                self.send_error(503, 'No fresh lowstate')
                return
            data = json.dumps(packet, allow_nan=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *unused):
            pass

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print('Read-only Go2 lowstate server on {}:{}'.format(args.bind, args.port), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        subscriber.Close()


def ros(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, Imu
    rclpy.init()
    node = Node('go2_live_joint_bridge')
    joints = node.create_publisher(JointState, '/joint_states', 10)
    imu = node.create_publisher(Imu, '/go2/imu', qos_profile_sensor_data)
    last = {'sequence': None, 'warning': 0.0}

    def poll():
        try:
            with urlopen(args.url, timeout=.5) as response:
                packet = json.loads(response.read(16384))
            if packet['sequence'] == last['sequence']:
                return
            for key in ('position', 'velocity', 'effort'):
                if len(packet[key]) != 12 or not all(math.isfinite(x) for x in packet[key]):
                    raise ValueError('Invalid motor state')
            stamp = node.get_clock().now().to_msg()
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = JOINTS
            msg.position, msg.velocity, msg.effort = (packet[k] for k in ('position', 'velocity', 'effort'))
            joints.publish(msg)
            body = Imu()
            body.header.stamp = stamp
            body.header.frame_id = 'base_link'
            q = packet['quaternion_wxyz']
            body.orientation.w, body.orientation.x, body.orientation.y, body.orientation.z = q
            for vector, key in ((body.angular_velocity, 'gyroscope'), (body.linear_acceleration, 'accelerometer')):
                vector.x, vector.y, vector.z = packet[key]
            # SDK supplies no covariance estimate; zero denotes unknown.
            imu.publish(body)
            last['sequence'] = packet['sequence']
        except Exception as exc:
            if time.monotonic() - last['warning'] > 5:
                node.get_logger().warning('Live joints unavailable: {}'.format(exc))
                last['warning'] = time.monotonic()

    node.create_timer(.05, poll)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    native = sub.add_parser('robot')
    native.add_argument('--interface', default='eth0')
    native.add_argument('--bind', default='192.168.123.18')
    native.add_argument('--port', type=int, default=8766)
    native.add_argument('--rate', type=float, default=100.0,
                        help='maximum lowstate conversion rate in Hz')
    local = sub.add_parser('ros')
    local.add_argument('--url', default='http://192.168.123.18:8766/state.json')
    args = parser.parse_args()
    (robot if args.mode == 'robot' else ros)(args)


if __name__ == '__main__':
    main()
