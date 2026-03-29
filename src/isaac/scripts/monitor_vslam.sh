#!/usr/bin/env bash
# Monitor Isaac VSLAM in real-time — run in a separate terminal (attach.sh)

set +u
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WS/src/install/setup.bash" ]; then source "$WS/src/install/setup.bash" || true; fi
set -u

echo "==============================="
echo "  ISAAC VSLAM MONITOR"
echo "==============================="
echo "Topics watched:"
echo "  - /visual_slam/tracking/odometry  (pose x,y,z)"
echo "  - /visual_slam/status             (tracking state)"
echo "  - /visual_slam/tracking/vo_pose   (visual odometry)"
echo ""
echo "Press Ctrl+C to stop"
echo "==============================="

python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import time

def quat_to_euler(x, y, z, w):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw

JUMP_THRESHOLD_M    = 0.1    # saut > 0.1m en une frame = ignoré
JUMP_THRESHOLD_DEG  = 10.0   # saut > 10° en une frame = ignoré
EXPLOSION_THRESHOLD = 1000.0 # pose > 1000m = tracking perdu
FREEZE_TIMEOUT_SEC  = 3.0    # pose figée > 3s = tracking gelé

class VslamMonitor(Node):
    def __init__(self):
        super().__init__('vslam_monitor')
        self._last_pose = None
        self._filtered_pose = None     # dernière position valide (filtrée)
        self._filtered_rot = None      # dernière rotation valide (filtrée) en degrés
        self._total_dist = 0.0
        self._filtered_dist = 0.0
        self._frame_count = 0
        self._skipped_count = 0
        self._lost_count = 0
        self._jump_count = 0
        self._freeze_count = 0
        self._last_update_time = None
        self._frozen_warned = False

        self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self._odom_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            '/visual_slam/tracking/vo_pose',
            self._vo_pose_cb,
            qos_profile_sensor_data,
        )

        # Timer to detect frozen tracking
        self.create_timer(1.0, self._check_freeze)

        self.get_logger().info('Monitor started — waiting for VSLAM topics...')
        self.get_logger().info(
            'Thresholds: jump=%.2fm  explosion=%.0fm  freeze=%.1fs'
            % (JUMP_THRESHOLD_M, EXPLOSION_THRESHOLD, FREEZE_TIMEOUT_SEC)
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._frame_count += 1
        now = time.time()
        self._last_update_time = now
        self._frozen_warned = False

        # 1. Detect explosion
        if abs(p.x) > EXPLOSION_THRESHOLD or abs(p.y) > EXPLOSION_THRESHOLD or abs(p.z) > EXPLOSION_THRESHOLD:
            self._lost_count += 1
            self.get_logger().error(
                '[TRACKING LOST #%d] pose exploded → x=%.1f  y=%.1f  z=%.1f'
                % (self._lost_count, p.x, p.y, p.z)
            )
            return

        # 2. Compute step from last raw pose
        step = 0.0
        if self._last_pose is not None:
            dx = p.x - self._last_pose[0]
            dy = p.y - self._last_pose[1]
            dz = p.z - self._last_pose[2]
            step = math.sqrt(dx*dx + dy*dy + dz*dz)
            self._total_dist += step

        self._last_pose = (p.x, p.y, p.z)

        # 3. Rotation
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)

        # 4. Filter position
        pos_jump = step > JUMP_THRESHOLD_M and self._filtered_pose is not None
        if pos_jump:
            self._jump_count += 1
            self._skipped_count += 1
            fp = self._filtered_pose
            self.get_logger().warn(
                '[POS FILTERED #%d] jump=+%.3fm ignored → keeping x=%+.3f y=%+.3f z=%+.3f'
                % (self._jump_count, step, fp[0], fp[1], fp[2])
            )
        else:
            if self._filtered_pose is not None:
                fdx = p.x - self._filtered_pose[0]
                fdy = p.y - self._filtered_pose[1]
                fdz = p.z - self._filtered_pose[2]
                self._filtered_dist += math.sqrt(fdx*fdx + fdy*fdy + fdz*fdz)
            self._filtered_pose = (p.x, p.y, p.z)

        # 5. Filter rotation
        rot_jump = False
        if self._filtered_rot is not None:
            dr = abs(roll  - self._filtered_rot[0])
            dp = abs(pitch - self._filtered_rot[1])
            dy = abs(yaw   - self._filtered_rot[2])
            # Handle wraparound (e.g. 179° → -179°)
            dr = min(dr, 360.0 - dr)
            dp = min(dp, 360.0 - dp)
            dy = min(dy, 360.0 - dy)
            rot_jump = max(dr, dp, dy) > JUMP_THRESHOLD_DEG

        if rot_jump:
            fr = self._filtered_rot
            self.get_logger().warn(
                '[ROT FILTERED] jump ignored → keeping rx=%+.1f° ry=%+.1f° rz=%+.1f°'
                % (fr[0], fr[1], fr[2])
            )
        else:
            self._filtered_rot = (roll, pitch, yaw)

        # 6. Print every 30 frames (~1s)
        fp = self._filtered_pose or (p.x, p.y, p.z)
        fr = self._filtered_rot or (roll, pitch, yaw)
        if self._frame_count % 30 == 0:
            self.get_logger().info(
                '[frame=%d] x=%+.3f y=%+.3f z=%+.3f | rx=%+.1f° ry=%+.1f° rz=%+.1f° | dist=%.2fm | skip=%d lost=%d frozen=%d'
                % (self._frame_count, fp[0], fp[1], fp[2],
                   fr[0], fr[1], fr[2],
                   self._filtered_dist, self._skipped_count, self._lost_count, self._freeze_count)
            )

    def _check_freeze(self):
        if self._last_update_time is None:
            return
        elapsed = time.time() - self._last_update_time
        if elapsed > FREEZE_TIMEOUT_SEC and not self._frozen_warned:
            self._freeze_count += 1
            self._frozen_warned = True
            self.get_logger().error(
                '[TRACKING FROZEN #%d] no pose update for %.1f s — SLAM lost tracking'
                % (self._freeze_count, elapsed)
            )

    def _vo_pose_cb(self, msg: PoseStamped):
        pass

def main():
    rclpy.init()
    node = VslamMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

main()
EOF
