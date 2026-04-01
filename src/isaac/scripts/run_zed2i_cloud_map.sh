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
from visualization_msgs.msg import Marker
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
        self.declare_parameter('map_frame',    'odom')
        self.declare_parameter('cloud_topic',  '/zed2i/zed_node/point_cloud/cloud_registered')

        self._voxel   = self.get_parameter('voxel_size').value
        self._maxpts  = self.get_parameter('max_points').value
        self._every   = self.get_parameter('update_every').value
        self._mframe  = self.get_parameter('map_frame').value
        self._topic   = self.get_parameter('cloud_topic').value

        self._accumulated = {}   # voxel key → (x, y, z, r, g, b)
        self._frame_count  = 0
        self._last_pos     = None  # last camera position for jump detection
        self._max_jump     = 0.3   # max allowed displacement per frame (m)

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(
            PointCloud2, self._topic, self._cloud_cb, qos_profile_sensor_data
        )

        map_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PointCloud2, '/cloud_map', map_qos)
        self._pub_filtered = self.create_publisher(PointCloud2, '/cloud_map_filtered', map_qos)
        self._pub_marker = self.create_publisher(Marker, '/camera_position', map_qos)
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

        # Jump detection: skip frame if camera moved too far since last frame
        if self._last_pos is not None:
            jump = np.linalg.norm(T - self._last_pos)
            if jump > self._max_jump:
                self.get_logger().warn(
                    'JUMP detected: %.3fm (max=%.2fm) — skipping frame %d'
                    % (jump, self._max_jump, self._frame_count))
                return
        self._last_pos = T.copy()

        # Read points (x, y, z, rgb)
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
        if not pts:
            return

        new_count = 0
        for pt in pts:
            x, y, z = pt[0], pt[1], pt[2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # Skip points beyond 3m from camera
            if math.sqrt(x*x + y*y + z*z) > 3.0:
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

        # Get camera position in map frame
        try:
            cam_tf = self._tf_buffer.lookup_transform(
                self._mframe, 'zed2i_left_camera_frame',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            cam_x = cam_tf.transform.translation.x
            cam_y = cam_tf.transform.translation.y
            cam_z = cam_tf.transform.translation.z
            cam_rot = cam_tf.transform.rotation
        except Exception:
            cam_x = cam_y = cam_z = 0.0
            cam_rot = None

        # Publish camera position marker
        marker = Marker()
        marker.header = header
        marker.ns = 'camera'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = cam_x
        marker.pose.position.y = cam_y
        marker.pose.position.z = cam_z
        if cam_rot:
            marker.pose.orientation = cam_rot
        else:
            marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3  # longueur flèche
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self._pub_marker.publish(marker)

        # Filtered: points between cam_z - 0.1m and cam_z + 0.1m
        z_min = cam_z - 0.1
        z_max = cam_z + 0.1
        filtered_pts = [(x, y, z, r, g, b) for (x, y, z, r, g, b) in pts if z_min <= z <= z_max]
        if filtered_pts:
            fdata = []
            for (x, y, z, r, g, b) in filtered_pts:
                rgb_int = (r << 16) | (g << 8) | b
                rgb_f   = struct.unpack('f', struct.pack('I', rgb_int))[0]
                fdata.append(struct.pack('ffff', x, y, z, rgb_f))

            filtered_msg = PointCloud2(
                header=header,
                height=1,
                width=len(filtered_pts),
                fields=fields,
                is_bigendian=False,
                point_step=16,
                row_step=16 * len(filtered_pts),
                data=b''.join(fdata),
                is_dense=True,
            )
            self._pub_filtered.publish(filtered_msg)

        self.get_logger().info(
            'Published /cloud_map: %d pts | /cloud_map_filtered: %d pts (cam @ %.2f,%.2f,%.2f, z±0.1m)'
            % (len(pts), len(filtered_pts) if filtered_pts else 0, cam_x, cam_y, cam_z),
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
