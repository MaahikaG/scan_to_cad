#!/home/mie_g28/venv/bin/python3
"""
motor_controller.py
───────────────────
Combined gantry + pan-tilt controller. Controls all four motors and
publishes scan position to ROS 2 so point_cloud_pub.py can build the
point cloud.

SCAN PATTERN:
  Gantry Motor 1 (servo, theta) sweeps 0° → 170° → 0° → ... in ~45° steps.
  At each ~45° stop, a full pan-tilt sweep is performed (Motor A × Motor B).
  After a complete one-way gantry sweep, Gantry Motor 2 (stepper, phi)
  advances by PHI_STEP_STEPS. Scan ends when phi reaches PHI_LIMIT_STEPS.

PAN-TILT SWEEP (at each gantry theta stop):
  Motor A (pan) steps through 0 → PT_A_STEPS in PT_A_INC increments.
  At each Motor A position, Motor B (tilt) sweeps 0 → PT_B_STEPS and back.
  PT_PAUSE_S is waited at each stop so the TOF sensor can capture a reading.

ROS TOPICS PUBLISHED:
  /odom  (nav_msgs/Odometry)
      position.{x,y,z}         = gantry scanner head 3D position (metres)
      twist.twist.linear.x      = gantry θ (degrees)
      twist.twist.linear.y      = gantry φ (degrees)
      twist.twist.linear.z      = pan-tilt Motor A angle (degrees)
      twist.twist.angular.x     = pan-tilt Motor B angle (degrees)
  /tf   (odom → scanner_head)

GPIO (BCM numbering):
  Gantry Motor 1 (servo, θ):   PWM=13
  Gantry Motor 2 (stepper, φ): STEP=23  DIR=24  EN=25
  Encoder 1 (gantry θ):        A=5   B=6
  Pan-tilt Motor A (pan):      STEP=16  DIR=20  EN=21
  Pan-tilt Motor B (tilt):     STEP=12  DIR=7   EN=8
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import RPi.GPIO as GPIO
import time
import threading
import math

# ── GPIO pins — gantry ────────────────────────────────────────────────────────
SERVO_PIN = 13
M2_STEP = 23;  M2_DIR = 24;  M2_EN = 25
ENC1_A = 5;    ENC1_B = 6

# ── GPIO pins — pan-tilt ──────────────────────────────────────────────────────
PA_STEP = 16;  PA_DIR = 20;  PA_EN = 21    # Motor A — pan
PB_STEP = 12;  PB_DIR = 7;   PB_EN = 8    # Motor B — tilt

# ── Servo constants ───────────────────────────────────────────────────────────
SERVO_FREQ          = 50
SERVO_MIN_PULSE     = 0.5
SERVO_MAX_PULSE     = 2.5
SERVO_RANGE_DEG     = 300.0

# ── Gantry stepper constants ──────────────────────────────────────────────────
STEP_DELAY          = 0.001

# ── Encoder constants (gantry theta only) ─────────────────────────────────────
PULSES_PER_REV      = 24
DEGREES_PER_COUNT   = 360.0 / (PULSES_PER_REV * 2)

# ── Gantry scan parameters ────────────────────────────────────────────────────
THETA_SWEEP_DEG     = 170.0         # one-way sweep angle
THETA_STEP_DEG      = 45.0          # stop every ~45° during sweep
PHI_STEP_STEPS      = 2909          # stepper steps per gantry sweep
                                    # = round(16000 * 30/165)
PHI_LIMIT_STEPS     = 16000         # total phi steps before scan ends
PHI_DEG_PER_STEP    = 180.0 / PHI_LIMIT_STEPS  # approximate arc degrees/step

# ── Pan-tilt sweep parameters ─────────────────────────────────────────────────
PT_STEP_DELAY       = 0.001         # seconds per pulse edge
PT_STEP_DEG         = 1.8           # degrees per full step (NEMA 8, full step)
PT_A_STEPS          = 400           # full range of Motor A (pan) — TUNE THIS
PT_B_STEPS          = 400           # full range of Motor B (tilt) — TUNE THIS
PT_A_INC            = 100           # Motor A steps per increment
PT_B_INC            = 100           # Motor B steps per increment
PT_PAUSE_S          = 0.15          # pause at each stop for TOF reading

# ── Servo control parameters ──────────────────────────────────────────────────
POSITION_TOL_DEG    = 8.0
SERVO_TIMEOUT       = 10.0
SERVO_SWEEP_TIME    = 4.0
SERVO_INCREMENT_DEG = 2.0

# ── Encoder debounce ──────────────────────────────────────────────────────────
DEBOUNCE_MS         = 3.0

# ── Physical constants ────────────────────────────────────────────────────────
ARC_RADIUS_M        = 0.30          # gantry arc radius in metres


class GantryController(Node):
    def __init__(self):
        super().__init__('gantry_controller')

        self._lock           = threading.Lock()
        self._enc1_count     = 0
        self._enc1_last_ms   = 0.0
        self._phi_steps_sent = 0
        self._pt_a_steps     = 0     # current pan Motor A position (steps from home)
        self._pt_b_steps     = 0     # current pan Motor B position (steps from home)

        # ── GPIO setup ────────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)

        # Servo (Gantry Motor 1)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self._pwm.start(0)

        # Gantry stepper (Motor 2)
        for pin in [M2_STEP, M2_DIR, M2_EN]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(M2_EN, GPIO.LOW)

        # Pan-tilt steppers (Motors A and B)
        for pin in [PA_STEP, PA_DIR, PA_EN, PB_STEP, PB_DIR, PB_EN]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(PA_EN, GPIO.LOW)
        GPIO.output(PB_EN, GPIO.LOW)

        # Encoder 1 (gantry theta)
        for pin in [ENC1_A, ENC1_B]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(ENC1_A, GPIO.BOTH, callback=self._enc1_cb)

        # ── ROS publishers ────────────────────────────────────────────────────
        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Gantry controller ready.')

    # ── Encoder callback ──────────────────────────────────────────────────────

    def _enc1_cb(self, channel):
        now = time.monotonic() * 1000.0
        if now - self._enc1_last_ms < DEBOUNCE_MS:
            return
        self._enc1_last_ms = now
        a = GPIO.input(ENC1_A)
        b = GPIO.input(ENC1_B)
        with self._lock:
            self._enc1_count += 1 if a != b else -1

    # ── Position properties ───────────────────────────────────────────────────

    @property
    def theta_deg(self):
        with self._lock:
            return self._enc1_count * DEGREES_PER_COUNT

    @property
    def phi_deg(self):
        return self._phi_steps_sent * PHI_DEG_PER_STEP

    # ── ROS publishing ────────────────────────────────────────────────────────

    def publish_pose(self, theta_g, phi_g, theta_pt, phi_pt):
        """
        Publish current scan position to /odom and /tf.
          theta_g, phi_g    — gantry angles in degrees
          theta_pt, phi_pt  — pan-tilt angles in degrees
        """
        now = self.get_clock().now().to_msg()

        t_g = math.radians(theta_g)
        p_g = math.radians(phi_g)
        x = ARC_RADIUS_M * math.cos(p_g) * math.cos(t_g)
        y = ARC_RADIUS_M * math.cos(p_g) * math.sin(t_g)
        z = ARC_RADIUS_M * math.sin(p_g)

        odom = Odometry()
        odom.header.stamp         = now
        odom.header.frame_id      = 'odom'
        odom.child_frame_id       = 'scanner_head'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w  = 1.0
        odom.twist.twist.linear.x  = theta_g   # gantry θ — used by point_cloud_pub
        odom.twist.twist.linear.y  = phi_g     # gantry φ — used by point_cloud_pub
        odom.twist.twist.linear.z  = theta_pt  # pan-tilt Motor A angle
        odom.twist.twist.angular.x = phi_pt    # pan-tilt Motor B angle
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp        = now
        t.header.frame_id     = 'odom'
        t.child_frame_id      = 'scanner_head'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w    = 1.0
        self.tf_broadcaster.sendTransform(t)

    # ── Low-level motor helpers ───────────────────────────────────────────────

    def _step(self, step_pin, dir_pin, n_steps, positive, delay=STEP_DELAY):
        GPIO.output(dir_pin, GPIO.HIGH if positive else GPIO.LOW)
        for _ in range(n_steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(delay)

    # ── Servo control ─────────────────────────────────────────────────────────

    def _angle_to_duty(self, angle):
        angle    = max(0.0, min(SERVO_RANGE_DEG, angle))
        pulse_ms = SERVO_MIN_PULSE + (angle / SERVO_RANGE_DEG) * \
                   (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
        return (pulse_ms / (1000.0 / SERVO_FREQ)) * 100.0

    def move_servo_to(self, target_deg):
        print(f"  Gantry θ → {target_deg:.1f}°")
        current = self.theta_deg
        delta   = target_deg - current
        steps   = max(1, int(round(abs(delta) / SERVO_INCREMENT_DEG)))
        delay   = (abs(delta) / THETA_SWEEP_DEG) * SERVO_SWEEP_TIME / steps
        for i in range(1, steps + 1):
            self._pwm.ChangeDutyCycle(
                self._angle_to_duty(current + delta * i / steps))
            time.sleep(delay)
        deadline = time.time() + SERVO_TIMEOUT
        while time.time() < deadline:
            if abs(self.theta_deg - target_deg) <= POSITION_TOL_DEG:
                print(f"  Gantry θ reached {self.theta_deg:.1f}°")
                return
            time.sleep(0.05)
        print(f"  Warning: servo did not confirm {target_deg:.1f}° "
              f"(encoder at {self.theta_deg:.1f}°) — continuing")

    # ── Gantry phi control ────────────────────────────────────────────────────

    def move_phi_steps(self, n_steps):
        print(f"  Gantry φ: {n_steps} steps "
              f"(total: {self._phi_steps_sent + n_steps})")
        self._step(M2_STEP, M2_DIR, n_steps, positive=True)
        self._phi_steps_sent += n_steps

    # ── Pan-tilt sweep ────────────────────────────────────────────────────────

    def _pan_tilt_sweep(self, theta_g):
        """
        Full 2D pan-tilt sweep at the current gantry theta position.
        Motor A steps through its range in PT_A_INC increments.
        At each A position, Motor B sweeps its full range and returns.
        Publishes /odom at every stop and pauses for TOF reading.
        """
        phi_g = self.phi_deg

        a_positions = list(range(0, PT_A_STEPS, PT_A_INC)) + [PT_A_STEPS]

        for a_target in a_positions:
            a_delta = a_target - self._pt_a_steps
            if a_delta != 0:
                self._step(PA_STEP, PA_DIR, abs(a_delta),
                           a_delta > 0, PT_STEP_DELAY)
                self._pt_a_steps = a_target
            theta_pt = self._pt_a_steps * PT_STEP_DEG

            b_positions = list(range(0, PT_B_STEPS, PT_B_INC)) + [PT_B_STEPS]
            for b_target in b_positions:
                b_delta = b_target - self._pt_b_steps
                if b_delta != 0:
                    self._step(PB_STEP, PB_DIR, abs(b_delta),
                               b_delta > 0, PT_STEP_DELAY)
                    self._pt_b_steps = b_target
                phi_pt = self._pt_b_steps * PT_STEP_DEG
                self.publish_pose(theta_g, phi_g, theta_pt, phi_pt)
                time.sleep(PT_PAUSE_S)

            # Return Motor B to home
            if self._pt_b_steps > 0:
                self._step(PB_STEP, PB_DIR, self._pt_b_steps,
                           False, PT_STEP_DELAY)
                self._pt_b_steps = 0

        # Return Motor A to home
        if self._pt_a_steps > 0:
            self._step(PA_STEP, PA_DIR, self._pt_a_steps,
                       False, PT_STEP_DELAY)
            self._pt_a_steps = 0

        self.publish_pose(theta_g, phi_g, 0.0, 0.0)

    # ── Sweep position generator ──────────────────────────────────────────────

    def _sweep_positions(self, from_deg, to_deg):
        """Stop positions from from_deg toward to_deg in THETA_STEP_DEG steps."""
        positions = []
        step = THETA_STEP_DEG if to_deg > from_deg else -THETA_STEP_DEG
        pos  = from_deg + step
        while (step > 0 and pos < to_deg) or (step < 0 and pos > to_deg):
            positions.append(round(pos, 1))
            pos += step
        positions.append(to_deg)
        return positions

    # ── Main scan loop ────────────────────────────────────────────────────────

    def run_scan(self):
        print("Homing servo to 0°...")
        self.move_servo_to(0.0)
        time.sleep(0.5)
        with self._lock:
            self._enc1_count = 0
        print("Encoder zeroed. Starting scan.\n")

        sweep_ends  = [THETA_SWEEP_DEG, 0.0]
        sweep_index = 0
        current_pos = 0.0

        while True:
            end       = sweep_ends[sweep_index % 2]
            positions = self._sweep_positions(current_pos, end)
            print(f"═══ Sweep {sweep_index + 1} │ "
                  f"{'→'.join(f'{p}°' for p in [current_pos] + positions)} │ "
                  f"φ steps={self._phi_steps_sent}")

            for pos in positions:
                self.move_servo_to(pos)
                print(f"  ── Pan-tilt sweep at gantry θ={pos}°")
                self._pan_tilt_sweep(pos)

            current_pos = end

            if self._phi_steps_sent >= PHI_LIMIT_STEPS:
                print("\nPhi limit reached. Scan complete.")
                break

            self.move_phi_steps(PHI_STEP_STEPS)
            sweep_index += 1

            if self._phi_steps_sent >= PHI_LIMIT_STEPS:
                end       = sweep_ends[sweep_index % 2]
                positions = self._sweep_positions(current_pos, end)
                print(f"\nCompleting final sweep to {end:.1f}°...")
                for pos in positions:
                    self.move_servo_to(pos)
                    print(f"  ── Pan-tilt sweep at gantry θ={pos}°")
                    self._pan_tilt_sweep(pos)
                print("\nScan complete.")
                break

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self):
        GPIO.remove_event_detect(ENC1_A)
        self._pwm.stop()
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.output(PA_EN, GPIO.HIGH)
        GPIO.output(PB_EN, GPIO.HIGH)
        GPIO.cleanup()


def main():
    rclpy.init()
    controller = GantryController()
    try:
        controller.run_scan()
        print("Scan complete. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        controller.cleanup()
        rclpy.shutdown()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
