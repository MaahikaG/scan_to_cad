"""
pan_tilt_controller.py
──────────────────────
Controls two NEMA 8 stepper motors via TB6600 drivers.
Motor A and Motor B are mechanically independent — each directly
rotates its own axis at a 1:1 ratio with no gearing.

COORDINATE SYSTEM:
  Motor A → θ (theta): rotation around the Z axis.
             Positive θ = counterclockwise when viewed from above.
             Negative θ = clockwise when viewed from above.

  Motor B → φ (phi): angular position along the semicircle arc.
             φ = 0°   → home end of the diameter (horizontal)
             φ = 90°  → top of the arc (directly above center, along +Z)
             φ = 180° → far end of the diameter (horizontal, opposite side)

Topics subscribed:
  /pan_tilt/theta_steps  (std_msgs/Float32)
      Steps to move Motor A (Z rotation).
      Positive = counterclockwise from above, negative = clockwise.

  /pan_tilt/phi_steps    (std_msgs/Float32)
      Steps to move Motor B (arc position).
      Positive = move from φ=0° toward φ=180° (through the top).
      Negative = move back toward φ=0°.

GPIO (BCM numbering):
  Motor A (theta):  STEP=16  DIR=20  EN=21
  Motor B (phi):    STEP=12  DIR=7   EN=8
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# ── GPIO pin assignments (BCM numbering) ──────────────────────────────────────
PA_STEP = 16;  PA_DIR = 20;  PA_EN = 21    # Motor A — theta (Z rotation)
PB_STEP = 12;  PB_DIR = 7;   PB_EN = 8    # Motor B — phi   (arc position)

STEP_DELAY = 0.001      # seconds per pulse edge → period = 2×STEP_DELAY
                        # 0.001 s → 500 steps/sec. Reduce to go faster.


class PanTiltController(Node):
    def __init__(self):
        super().__init__('pan_tilt_controller')

        # ── GPIO setup ────────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)
        for pin in [PA_STEP, PA_DIR, PA_EN, PB_STEP, PB_DIR, PB_EN]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        # Enable both drivers (EN is active LOW on TB6600)
        GPIO.output(PA_EN, GPIO.LOW)
        GPIO.output(PB_EN, GPIO.LOW)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float32, '/pan_tilt/theta_steps', self.theta_cb, 10
        )
        self.create_subscription(
            Float32, '/pan_tilt/phi_steps', self.phi_cb, 10
        )

        self.get_logger().info(
            'Pan-tilt controller ready | '
            '/pan_tilt/theta_steps → Motor A (Z rotation) | '
            '/pan_tilt/phi_steps   → Motor B (arc position)'
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _move(self, step_pin: int, dir_pin: int, steps: int):
        """
        Send `steps` pulses to one motor.
        Positive steps → DIR HIGH (counterclockwise / toward φ=180°).
        Negative steps → DIR LOW  (clockwise / toward φ=0°).
        """
        if steps == 0:
            return
        GPIO.output(dir_pin, GPIO.HIGH if steps > 0 else GPIO.LOW)
        for _ in range(abs(steps)):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(STEP_DELAY)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(STEP_DELAY)

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def theta_cb(self, msg: Float32):
        """Move Motor A — Z-axis rotation (theta)."""
        self._move(PA_STEP, PA_DIR, int(msg.data))

    def phi_cb(self, msg: Float32):
        """Move Motor B — arc position along semicircle (phi)."""
        self._move(PB_STEP, PB_DIR, int(msg.data))

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        GPIO.output(PA_EN, GPIO.HIGH)
        GPIO.output(PB_EN, GPIO.HIGH)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
