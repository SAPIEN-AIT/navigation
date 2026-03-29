#!/usr/bin/env bash
# Accumulate ZED point cloud in the map frame using VSLAM pose.
# Launch AFTER run_zed2i_vslam.sh is running.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SCRIPT_DIR/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ];     then source "$WS/install/setup.bash"     || true; fi
if [ -f "$WS/src/install/setup.bash" ]; then source "$WS/src/install/setup.bash" || true; fi
set -u

printf 'Launching 3D point cloud map accumulator...\n'
printf 'Accumulated map topic: /cloud_map\n'
printf 'Make sure run_zed2i_vslam.sh is running first.\n\n'

python3 - "$@" <<'EOF'
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
import tf2_py
from geometry_msgs.msg import TransformStamped
import struct

VOXEL_SIZE   = 0.05   # m — résolution de la map (plus petit = plus dense mais plus lourd)
MAX_POINTS   = 2_000_000
UPDATE_EVERY = 5       # accumuler 1 frame sur N (économise CPU)

class CloudMapAccumulator(Node):
    def __init__(self):
        super().__init__('cloud_map_accumulator')
        self.declare_parameter('voxel_size',   VOXEL_SIZE)
        self.declare_parameter('max_points',   MAX_POINTS)
        self.declare_parameter('update_every', UPDATE_EVERY)
        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('cloud_topic',  '/zed2i/zed_node/point_cloud/cloud_registered')

        self._voxel   = self.get_parameter('voxel_size').value
        self._maxpts  = self.get_parameter('max_points').value
        self._every   = self.get_parameter('update_every').value
        self._mframe  = self.get_parameter('map_frame').value
        self._topic   = self.get_parameter('cloud_topic').value

        self._accumulated = {}   # voxel key → (x, y, z, r, g, b)
        self._frame_count  = 0

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            PointCloud2, self._topic, self._cloud_cb, qos_profile_sensor_data
        )

        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PointCloud2, '/cloud_map', map_qos)
        self.create_timer(1.0, self._publish_map)

        self.get_logger().info(
            'Accumulating %s → /cloud_map (voxel=%.2fm, max=%d pts, 1 frame/%d)'
            % (self._topic, self._voxel, self._maxpts, self._every)
        )

    def _cloud_cb(self, msg: PointCloud2):
        self._frame_count += 1
        if self._frame_count % self._every != 0:
            return

        # Get transform: camera_frame → map
        try:
            tf = self._tf_buffer.lookup_transform(
                self._mframe,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn('TF not available: %s' % str(e), throttle_duration_sec=3.0)
            return

        tx = tf.transform.translation
        r  = tf.transform.rotation
        T  = np.array([tx.x, tx.y, tx.z])
        R  = self._quat_to_rot(r.x, r.y, r.z, r.w)

        # Read points (x, y, z, rgb)
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
        if not pts:
            return

        new_count = 0
        for pt in pts:
            x, y, z = pt[0], pt[1], pt[2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            # Transform to map frame
            p = R @ np.array([x, y, z]) + T

            # Voxel key
            key = (
                int(p[0] / self._voxel),
                int(p[1] / self._voxel),
                int(p[2] / self._voxel),
            )

            if key not in self._accumulated:
                # Decode RGB
                rgb_int = struct.unpack('I', struct.pack('f', pt[3]))[0]
                r8 = (rgb_int >> 16) & 0xFF
                g8 = (rgb_int >> 8)  & 0xFF
                b8 =  rgb_int        & 0xFF
                self._accumulated[key] = (p[0], p[1], p[2], r8, g8, b8)
                new_count += 1

        if new_count > 0:
            self.get_logger().info(
                'Frame %d: +%d new voxels → total=%d'
                % (self._frame_count, new_count, len(self._accumulated))
            )

        # Trim if too large
        if len(self._accumulated) > self._maxpts:
            excess = len(self._accumulated) - self._maxpts
            for k in list(self._accumulated.keys())[:excess]:
                del self._accumulated[k]

    def _publish_map(self):
        if not self._accumulated:
            return

        pts = list(self._accumulated.values())
        header = Header()
        header.frame_id = self._mframe
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = []
        for (x, y, z, r, g, b) in pts:
            rgb_int = (r << 16) | (g << 8) | b
            rgb_f   = struct.unpack('f', struct.pack('I', rgb_int))[0]
            data.append(struct.pack('ffff', x, y, z, rgb_f))

        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(pts),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(pts),
            data=b''.join(data),
            is_dense=True,
        )
        self._pub.publish(cloud_msg)
        self.get_logger().info(
            'Published /cloud_map: %d points' % len(pts),
            throttle_duration_sec=3.0,
        )

    @staticmethod
    def _quat_to_rot(qx, qy, qz, qw):
        return np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
            [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
        ])


def main():
    rclpy.init(args=sys.argv[1:] if len(sys.argv) > 1 else None)
    node = CloudMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()
EOF
