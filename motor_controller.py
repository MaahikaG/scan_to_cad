"""
motor_controller.py
───────────────────
Controls the gantry:
  Motor 1 → θ (theta): ASMC-04B servo, rotation around Z axis.
             Controlled by absolute angle (0°–300°).
             0° and 300° are the physical limits of the servo.

  Motor 2 → φ (phi): stepper motor via TB6600, angular position
             along the semicircle arc.
             φ = 0°   → home end of the diameter (horizontal)
             φ = 90°  → top of the arc (directly above center, along +Z)
             φ = 180° → far end of the diameter (horizontal, opposite side)

Topics subscribed:
  /motor/theta_deg    (std_msgs/Float32)
      Absolute angle to move servo to (0°–300°).

  /motor/phi_steps    (std_msgs/Float32)
      Steps to move Motor 2 (arc position).
      Positive = move from φ=0° toward φ=180° (through the top).
      Negative = move back toward φ=0°.

GPIO (BCM numbering):
  Motor 1 (theta):  SERVO=13  (hardware PWM)
  Motor 2 (phi):    STEP=23   DIR=24   EN=25
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# ── GPIO pin assignments (BCM numbering) ──────────────────────────────────────
SERVO_PIN = 13                  # Motor 1 — theta (Z rotation), hardware PWM

M2_STEP = 23;  M2_DIR = 24;  M2_EN = 25    # Motor 2 — phi (arc position)

# ── Servo constants ───────────────────────────────────────────────────────────
SERVO_FREQ      = 50            # Hz — standard servo PWM frequency
SERVO_MIN_PULSE = 0.5           # ms — pulse width at 0°
SERVO_MAX_PULSE = 2.5           # ms — pulse width at 300°
SERVO_MAX_ANGLE = 300.0         # degrees — full range of ASMC-04B

# ── Stepper constants ─────────────────────────────────────────────────────────
STEP_DELAY = 0.001              # seconds per pulse edge → 500 steps/sec


def _angle_to_duty(angle: float) -> float:
    """Convert servo angle (0°–300°) to PWM duty cycle (%)."""
    angle = max(0.0, min(SERVO_MAX_ANGLE, angle))
    pulse_ms = SERVO_MIN_PULSE + (angle / SERVO_MAX_ANGLE) * \
               (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
    return (pulse_ms / (1000.0 / SERVO_FREQ)) * 100.0


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # ── GPIO setup ────────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)

        # Servo (Motor 1)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self._servo_pwm.start(0)

        # Stepper (Motor 2)
        for pin in [M2_STEP, M2_DIR, M2_EN]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(M2_EN, GPIO.LOW)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            Float32, '/motor/theta_deg', self.theta_cb, 10
        )
        self.create_subscription(
            Float32, '/motor/phi_steps', self.phi_cb, 10
        )

        self.get_logger().info(
            'Motor controller ready | '
            '/motor/theta_deg  → Servo Motor 1 (Z rotation, 0°–300°) | '
            '/motor/phi_steps  → Stepper Motor 2 (arc position)'
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _move(self, step_pin: int, dir_pin: int, steps: int):
        """
        Send `steps` pulses to Motor 2.
        Positive steps → DIR HIGH (toward φ=180°).
        Negative steps → DIR LOW  (toward φ=0°).
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
        """Move servo to absolute angle (0°–300°)."""
        self._servo_pwm.ChangeDutyCycle(_angle_to_duty(msg.data))

    def phi_cb(self, msg: Float32):
        """Move Motor 2 — arc position along semicircle (phi)."""
        self._move(M2_STEP, M2_DIR, int(msg.data))

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._servo_pwm.stop()
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
