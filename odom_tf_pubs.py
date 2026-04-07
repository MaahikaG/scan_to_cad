"""
odom_tf_pubs.py
───────────────
Reads both rotary encoders and publishes the scanner head's position using
a spherical coordinate system matched to the physical mechanism.

GEOMETRY:
  The semicircle's diameter lies flat in the XY plane, passing through
  the Z axis. The arc bulges upward (+Z). Motor 1 rotates the entire
  semicircle around the Z axis (θ). Motor 2 drives the scanner head
  along the arc (φ).

  φ = 0°   → home end of diameter, horizontal  → (R, 0, 0) rotated by θ
  φ = 90°  → top of arc, directly above center → (0, 0, R)
  φ = 180° → opposite end of diameter          → (-R, 0, 0) rotated by θ

CARTESIAN CONVERSION:
  x = R · cos(φ) · cos(θ)
  y = R · cos(φ) · sin(θ)
  z = R · sin(φ)

Topics published:
  /odom  (nav_msgs/Odometry)
      position.{x,y,z}      = 3D world-space position of scanner head (metres)
      twist.twist.linear.x  = θ in degrees  (raw, for point_cloud_pub.py)
      twist.twist.linear.y  = φ in degrees  (raw, for point_cloud_pub.py)

  /tf   (via tf2_ros.TransformBroadcaster)
      'odom' → 'scanner_head'
      Used by point_cloud_pub.py to place TOF readings in world space.

GPIO (BCM numbering):
  Encoder 1 (Motor 1, θ):  A=5   B=6
  Encoder 2 (Motor 2, φ):  A=14  B=19

Constants to set before running:
  PULSES_PER_REV  — encoder PPR from datasheet
  ARC_RADIUS_M    — distance from Z axis to scanner head along arm (metres)
  PHI_HOME_DEG    — φ at startup; 0.0 if homed to the φ=0 end of diameter
  THETA_HOME_DEG  — θ at startup; almost always 0.0
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import RPi.GPIO as GPIO
import math

# ── GPIO pin assignments (BCM numbering) ──────────────────────────────────────
ENC1_A = 5;   ENC1_B = 6    # Encoder 1 — Motor 1 (θ, Z rotation)
ENC2_A = 14;  ENC2_B = 19   # Encoder 2 — Motor 2 (φ, arc position)

# ── Physical constants — TUNE THESE TO YOUR HARDWARE ──────────────────────────
PULSES_PER_REV = 600        # Encoder pulses per full shaft revolution (PPR).
                            # From encoder datasheet. Common: 100, 200, 400, 600.
                            # x2 quadrature decoding applied in code →
                            # effective resolution = PULSES_PER_REV * 2.

ARC_RADIUS_M   = 0.30       # Distance from the Z axis to the scanner head
                            # mounting point, measured along the arm (metres).
                            # This is the radius of the semicircle.
                            # Imported by point_cloud_pub.py — only set it here.

PHI_HOME_DEG   = 0.0        # φ angle at startup (degrees).
                            # 0.0 = head starts at home end of diameter.

THETA_HOME_DEG = 0.0        # θ angle at startup. Almost always 0.0.

# ── Derived constant ──────────────────────────────────────────────────────────
DEGREES_PER_COUNT = 360.0 / (PULSES_PER_REV * 2)

# Exported so point_cloud_pub.py can import ARC_RADIUS_M directly,
# keeping the value defined in exactly one place.
__all__ = ['ARC_RADIUS_M']


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_pubs')

        # ── Encoder state ─────────────────────────────────────────────────────
        self.enc1_count = 0
        self.enc2_count = 0

        # ── GPIO setup ────────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        for pin in [ENC1_A, ENC1_B, ENC2_A, ENC2_B]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Interrupt on BOTH edges of channel A → x2 quadrature decoding
        GPIO.add_event_detect(ENC1_A, GPIO.BOTH, callback=self._enc1_cb)
        GPIO.add_event_detect(ENC2_A, GPIO.BOTH, callback=self._enc2_cb)

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

    # ── Quadrature decoding ───────────────────────────────────────────────────

    def _enc1_cb(self, channel):
        """
        x2 quadrature decode — Motor 1 (θ).
        A leads B → counterclockwise → positive θ → count up.
        B leads A → clockwise → negative θ → count down.
        """
        a = GPIO.input(ENC1_A)
        b = GPIO.input(ENC1_B)
        self.enc1_count += 1 if a != b else -1

    def _enc2_cb(self, channel):
        """
        x2 quadrature decode — Motor 2 (φ).
        A leads B → moving from φ=0° toward φ=180° → count up.
        B leads A → moving back toward φ=0° → count down.
        """
        a = GPIO.input(ENC2_A)
        b = GPIO.input(ENC2_B)
        self.enc2_count += 1 if a != b else -1

    # ── Coordinate conversion ─────────────────────────────────────────────────

    def _counts_to_angles(self):
        """Convert raw encoder counts to (theta_deg, phi_deg)."""
        theta_deg = THETA_HOME_DEG + self.enc1_count * DEGREES_PER_COUNT
        phi_deg   = PHI_HOME_DEG   + self.enc2_count * DEGREES_PER_COUNT
        return theta_deg, phi_deg

    @staticmethod
    def spherical_to_cartesian(theta_deg: float, phi_deg: float, radius: float):
        """
        Convert (θ, φ) to 3D Cartesian (x, y, z).

          φ=0°:   (R·cosθ,  R·sinθ,  0)   — home end of diameter
          φ=90°:  (0,       0,        R)   — top of arc
          φ=180°: (-R·cosθ, -R·sinθ, 0)   — far end of diameter

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

    # ── Publish loop ──────────────────────────────────────────────────────────

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()

        theta_deg, phi_deg = self._counts_to_angles()
        x, y, z = self.spherical_to_cartesian(theta_deg, phi_deg, ARC_RADIUS_M)

        # ── Odometry message ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'scanner_head'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = 1.0
        # Pack raw angles into unused twist fields for point_cloud_pub.py
        odom.twist.twist.linear.x = theta_deg   # θ in degrees
        odom.twist.twist.linear.y = phi_deg     # φ in degrees
        self.odom_pub.publish(odom)

        # ── TF: odom → scanner_head ───────────────────────────────────────────
        t = TransformStamped()
        t.header.stamp       = now
        t.header.frame_id    = 'odom'
        t.child_frame_id     = 'scanner_head'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w    = 1.0
        self.tf_broadcaster.sendTransform(t)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        # Remove event detects before GPIO cleanup to avoid runtime warnings
        GPIO.remove_event_detect(ENC1_A)
        GPIO.remove_event_detect(ENC2_A)
        GPIO.cleanup()
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