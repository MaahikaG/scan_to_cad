"""
odom_tf_pubs.py
───────────────
Publishes the scanner mount's position along the gantry arc and broadcasts
the TF chain. Theta and phi are received from motor_controller via /odom_raw
instead of reading GPIO encoders directly — this avoids GPIO pin conflicts
since motor_controller already owns the encoder pins.

GEOMETRY:
  The semicircle's diameter lies flat in the XY plane, passing through
  the Z axis. The arc bulges upward (+Z). Motor 1 rotates the entire
  semicircle around the Z axis (θ). Motor 2 drives the scanner mount
  along the arc (φ).

  φ = 0°   → home end of diameter, horizontal  → (R, 0, 0) rotated by θ
  φ = 90°  → top of arc, directly above center → (0, 0, R)
  φ = 180° → opposite end of diameter          → (-R, 0, 0) rotated by θ

CARTESIAN CONVERSION (gantry position):
  x = R · cos(φ) · cos(θ)
  y = R · cos(φ) · sin(θ)
  z = R · sin(φ)

SCANNER HEAD LOCAL FRAME:
  The frame 'scanner_head' is oriented so that:
    x-axis = -e_r  (inward toward scan target; sensor zero direction)
    y-axis = -e_θ  (lateral)
    z-axis =  e_φ  (upward along the arc)
  This is the parent frame for the pan-tilt rotation.

TF CHAIN:
  odom → scanner_head : gantry position + mount orientation (from θ, φ)
  scanner_head → sensor : pan-tilt rotation (α pan around z, β tilt around y)

Topics published:
  /odom  (nav_msgs/Odometry)
      pose.pose.position.{x,y,z}     = 3D position of scanner mount (metres)
      pose.pose.orientation          = scanner head orientation (from θ, φ)
      twist.twist.linear.x           = θ in degrees  (raw, for point_cloud_pub)
      twist.twist.linear.y           = φ in degrees  (raw, for point_cloud_pub)

  /tf   (via tf2_ros.TransformBroadcaster)
      'odom'         → 'scanner_head'  (gantry position + mount orientation)
      'scanner_head' → 'sensor'        (pan-tilt rotation)

Topics subscribed:
  /odom_raw          (std_msgs/Float32MultiArray)
      data[0] = θ in degrees  (from motor_controller)
      data[1] = φ in degrees  (from motor_controller)

  /pan_tilt/angles   (geometry_msgs/Vector3)
      x = α (pan)  in degrees
      y = β (tilt) in degrees

Constants to set before running:
  PULSES_PER_REV  — encoder PPR from datasheet (used only for deg/count display)
  ARC_RADIUS_M    — distance from Z axis to scanner mount along arm (metres)
  PHI_HOME_DEG    — φ at startup; 0.0 if homed to the φ=0 end of diameter
  THETA_HOME_DEG  — θ at startup; almost always 0.0
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import Float32MultiArray
import tf2_ros
import math

# ── Physical constants — TUNE THESE TO YOUR HARDWARE ──────────────────────────
PULSES_PER_REV = 600

ARC_RADIUS_M   = 0.30       # Distance from the Z axis to the scanner mount
                            # point, measured along the arm (metres).
                            # Imported by point_cloud_pub.py — only set it here.

PHI_HOME_DEG   = 0.0
THETA_HOME_DEG = 0.0

# ── Derived constant ──────────────────────────────────────────────────────────
DEGREES_PER_COUNT = 360.0 / (PULSES_PER_REV * 2)

# Exported so point_cloud_pub.py can import ARC_RADIUS_M directly
__all__ = ['ARC_RADIUS_M']


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_pubs')

        # ── Gantry angle state (from /odom_raw) ───────────────────────────────
        self.theta_deg = THETA_HOME_DEG
        self.phi_deg   = PHI_HOME_DEG

        # ── Pan-tilt state (from /pan_tilt/angles) ────────────────────────────
        self.alpha_deg = 0.0
        self.beta_deg  = 0.0

        # ── ROS subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, '/odom_raw', self._odom_raw_cb, 10)
        self.create_subscription(
            Vector3, '/pan_tilt/angles', self._pan_tilt_cb, 10)

        # ── ROS publishers ────────────────────────────────────────────────────
        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publish at 20 Hz
        self.create_timer(0.05, self.publish_odometry)

        self.get_logger().info(
            f'Odom/TF publisher ready | '
            f'R={ARC_RADIUS_M} m | '
            f'{PULSES_PER_REV} PPR | '
            f'{DEGREES_PER_COUNT:.4f} deg/count'
        )

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _odom_raw_cb(self, msg: Float32MultiArray):
        """Receive theta and phi in degrees from motor_controller."""
        if len(msg.data) >= 2:
            self.theta_deg = msg.data[0]
            self.phi_deg   = msg.data[1]

    def _pan_tilt_cb(self, msg: Vector3):
        """Receive pan (α) and tilt (β) angles in degrees from motor_controller."""
        self.alpha_deg = msg.x
        self.beta_deg  = msg.y

    # ── Coordinate conversion ─────────────────────────────────────────────────

    @staticmethod
    def spherical_to_cartesian(theta_deg: float, phi_deg: float, radius: float):
        """
        Convert (θ, φ) to 3D Cartesian (x, y, z).

          x = R · cos(φ) · cos(θ)
          y = R · cos(φ) · sin(θ)
          z = R · sin(φ)
        """
        theta = math.radians(theta_deg)
        phi   = math.radians(phi_deg)
        x = radius * math.cos(phi) * math.cos(theta)
        y = radius * math.cos(phi) * math.sin(theta)
        z = radius * math.sin(phi)
        return x, y, z

    @staticmethod
    def _mount_orientation_quat(theta_deg: float, phi_deg: float):
        """
        Quaternion for the 'scanner_head' frame orientation in the 'odom' frame.
        Uses Shepperd's method. Returns (qx, qy, qz, qw).
        """
        θ = math.radians(theta_deg)
        φ = math.radians(phi_deg)

        R = [
            [-math.cos(φ)*math.cos(θ),  math.sin(θ), -math.sin(φ)*math.cos(θ)],
            [-math.cos(φ)*math.sin(θ), -math.cos(θ), -math.sin(φ)*math.sin(θ)],
            [-math.sin(φ),              0.0,           math.cos(φ)             ],
        ]

        trace = R[0][0] + R[1][1] + R[2][2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2][1] - R[1][2]) * s
            y = (R[0][2] - R[2][0]) * s
            z = (R[1][0] - R[0][1]) * s
        elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
            s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
            w = (R[2][1] - R[1][2]) / s
            x = 0.25 * s
            y = (R[0][1] + R[1][0]) / s
            z = (R[0][2] + R[2][0]) / s
        elif R[1][1] > R[2][2]:
            s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
            w = (R[0][2] - R[2][0]) / s
            x = (R[0][1] + R[1][0]) / s
            y = 0.25 * s
            z = (R[1][2] + R[2][1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
            w = (R[1][0] - R[0][1]) / s
            x = (R[0][2] + R[2][0]) / s
            y = (R[1][2] + R[2][1]) / s
            z = 0.25 * s
        return x, y, z, w

    @staticmethod
    def _pan_tilt_quat(alpha_deg: float, beta_deg: float):
        """
        Quaternion for the 'sensor' frame relative to 'scanner_head'.
        Returns (qx, qy, qz, qw).
        """
        a = math.radians(alpha_deg)
        b = math.radians(beta_deg)
        sa, ca = math.sin(a / 2), math.cos(a / 2)
        sb, cb = math.sin(b / 2), math.cos(b / 2)
        return (-sa * sb,
                 ca * sb,
                 sa * cb,
                 ca * cb)

    # ── Publish loop ──────────────────────────────────────────────────────────

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()

        theta_deg = self.theta_deg
        phi_deg   = self.phi_deg
        x, y, z   = self.spherical_to_cartesian(theta_deg, phi_deg, ARC_RADIUS_M)
        qx, qy, qz, qw = self._mount_orientation_quat(theta_deg, phi_deg)

        # ── Odometry ──────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'scanner_head'
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = theta_deg
        odom.twist.twist.linear.y = phi_deg
        self.odom_pub.publish(odom)

        # ── TF 1: odom → scanner_head ─────────────────────────────────────────
        t1 = TransformStamped()
        t1.header.stamp       = now
        t1.header.frame_id    = 'odom'
        t1.child_frame_id     = 'scanner_head'
        t1.transform.translation.x = x
        t1.transform.translation.y = y
        t1.transform.translation.z = z
        t1.transform.rotation.x = qx
        t1.transform.rotation.y = qy
        t1.transform.rotation.z = qz
        t1.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t1)

        # ── TF 2: scanner_head → sensor ───────────────────────────────────────
        pqx, pqy, pqz, pqw = self._pan_tilt_quat(self.alpha_deg, self.beta_deg)

        t2 = TransformStamped()
        t2.header.stamp       = now
        t2.header.frame_id    = 'scanner_head'
        t2.child_frame_id     = 'sensor'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.x = pqx
        t2.transform.rotation.y = pqy
        t2.transform.rotation.z = pqz
        t2.transform.rotation.w = pqw
        self.tf_broadcaster.sendTransform(t2)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()