"""
motor_controller.py
───────────────────
Standalone gantry controller — no ROS required.
Run directly: python3 motor_controller.py

SCAN PATTERN:
  Motor 1 (theta, servo)  sweeps 0° → 180° → 0° → 180° ... repeatedly.
  Motor 2 (phi, stepper)  advances PHI_STEP_DEG after every one-way sweep.
  Scan ends when phi reaches PHI_LIMIT_DEG — Motor 1 completes one final
  180° sweep, then both motors hold position.

STALL DETECTION:
  Motor 1 (servo): commanded once per sweep. If encoder does not confirm
                   arrival within SERVO_TIMEOUT seconds, logs a warning
                   and continues to the next step.
  Motor 2 (stepper): retries until encoder confirms position within
                     POSITION_TOL_DEG, up to MAX_PHI_RETRIES attempts.

GPIO (BCM numbering):
  Motor 1 (theta, servo): PWM=13
  Motor 2 (phi, stepper): STEP=23  DIR=24  EN=25
  Encoder 1 (theta):      A=5   B=6
  Encoder 2 (phi):        A=14  B=19
"""

import RPi.GPIO as GPIO
import time
import threading

# ── GPIO pins ─────────────────────────────────────────────────────────────────
SERVO_PIN = 13
M2_STEP = 23;  M2_DIR = 24;  M2_EN = 25
ENC1_A = 5;    ENC1_B = 6
ENC2_A = 14;   ENC2_B = 19

# ── Servo constants ───────────────────────────────────────────────────────────
SERVO_FREQ      = 50            # Hz
SERVO_MIN_PULSE = 0.5           # ms — pulse width at 0°
SERVO_MAX_PULSE = 2.5           # ms — pulse width at 300°
SERVO_RANGE_DEG = 300.0         # full mechanical range of ASMC-04B

# ── Stepper constants ─────────────────────────────────────────────────────────
STEP_DELAY     = 0.001          # seconds per pulse edge → 500 steps/sec
STEP_ANGLE_DEG = 1.8            # degrees per full step (full step mode)

# ── Encoder constants ─────────────────────────────────────────────────────────
PULSES_PER_REV    = 24          # encoder PPR — PEL12T: 24 pulses per 360°
DEGREES_PER_COUNT = 360.0 / (PULSES_PER_REV * 2)   # x2 quadrature decoding

# ── Scan parameters — tune these to your hardware ─────────────────────────────
THETA_SWEEP_DEG = 180.0         # one-way sweep angle for Motor 1
PHI_STEP_DEG    = 15.0          # phi advance per Motor 1 sweep
PHI_LIMIT_DEG   = 120.0         # end-of-arc limit for Motor 2

# ── Control parameters ────────────────────────────────────────────────────────
POSITION_TOL_DEG = 8.0          # acceptable position error in degrees
                                # must be >= DEGREES_PER_COUNT (7.5° at 24 PPR)
SERVO_TIMEOUT    = 10.0         # seconds to wait for servo encoder confirmation
SERVO_SWEEP_TIME = 1.5          # seconds for a full 180° sweep
SERVO_INCREMENT_DEG = 2.0       # degrees per increment — smaller = smoother
MAX_PHI_RETRIES  = 10           # max stall retries for Motor 2


class GantryController:
    def __init__(self):
        self._lock = threading.Lock()
        self._enc1_count = 0
        self._enc2_count = 0

        # ── GPIO setup ────────────────────────────────────────────────────────
        GPIO.setmode(GPIO.BCM)

        # Servo (Motor 1)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self._pwm.start(0)

        # Stepper (Motor 2)
        for pin in [M2_STEP, M2_DIR, M2_EN]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(M2_EN, GPIO.LOW)

        # Encoders
        for pin in [ENC1_A, ENC1_B, ENC2_A, ENC2_B]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(ENC1_A, GPIO.BOTH, callback=self._enc1_cb)
        GPIO.add_event_detect(ENC2_A, GPIO.BOTH, callback=self._enc2_cb)

    # ── Encoder callbacks (run on GPIO interrupt thread) ──────────────────────

    def _enc1_cb(self, channel):
        a = GPIO.input(ENC1_A)
        b = GPIO.input(ENC1_B)
        with self._lock:
            self._enc1_count += 1 if a != b else -1

    def _enc2_cb(self, channel):
        a = GPIO.input(ENC2_A)
        b = GPIO.input(ENC2_B)
        with self._lock:
            self._enc2_count += 1 if a != b else -1

    # ── Position properties ───────────────────────────────────────────────────

    @property
    def theta_deg(self):
        with self._lock:
            return self._enc1_count * DEGREES_PER_COUNT

    @property
    def phi_deg(self):
        with self._lock:
            return self._enc2_count * DEGREES_PER_COUNT

    def _reset_encoders(self):
        with self._lock:
            self._enc1_count = 0
            self._enc2_count = 0

    # ── Servo control ─────────────────────────────────────────────────────────

    def _angle_to_duty(self, angle):
        angle = max(0.0, min(SERVO_RANGE_DEG, angle))
        pulse_ms = SERVO_MIN_PULSE + (angle / SERVO_RANGE_DEG) * \
                   (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
        return (pulse_ms / (1000.0 / SERVO_FREQ)) * 100.0

    def move_servo_to(self, target_deg):
        """
        Command servo to target_deg in small increments to limit speed.
        One attempt only — logs a warning and continues if not reached.
        SERVO_SWEEP_TIME controls how long a full 180° sweep takes.
        """
        print(f"  Theta → {target_deg:.1f}°")
        current = self.theta_deg
        delta = target_deg - current
        steps = max(1, int(round(abs(delta) / SERVO_INCREMENT_DEG)))
        delay = (abs(delta) / THETA_SWEEP_DEG) * SERVO_SWEEP_TIME / steps

        for i in range(1, steps + 1):
            intermediate = current + (delta * i / steps)
            self._pwm.ChangeDutyCycle(self._angle_to_duty(intermediate))
            time.sleep(delay)

        deadline = time.time() + SERVO_TIMEOUT
        while time.time() < deadline:
            if abs(self.theta_deg - target_deg) <= POSITION_TOL_DEG:
                print(f"  Theta reached {self.theta_deg:.1f}°")
                return
            time.sleep(0.05)
        print(f"  Warning: servo did not confirm {target_deg:.1f}° "
              f"(encoder at {self.theta_deg:.1f}°) — continuing")

    # ── Stepper control ───────────────────────────────────────────────────────

    def _send_steps(self, n_steps, positive):
        GPIO.output(M2_DIR, GPIO.HIGH if positive else GPIO.LOW)
        for _ in range(n_steps):
            GPIO.output(M2_STEP, GPIO.HIGH)
            time.sleep(STEP_DELAY)
            GPIO.output(M2_STEP, GPIO.LOW)
            time.sleep(STEP_DELAY)

    def move_phi_by(self, delta_deg):
        """
        Advance phi by delta_deg using encoder feedback.
        Retries on stall up to MAX_PHI_RETRIES times.
        """
        target = self.phi_deg + delta_deg
        positive = delta_deg > 0
        print(f"  Phi → {target:.1f}°")

        for attempt in range(MAX_PHI_RETRIES):
            remaining = target - self.phi_deg
            if abs(remaining) <= POSITION_TOL_DEG:
                break
            steps = max(1, int(round(abs(remaining) / STEP_ANGLE_DEG)))
            before = self.phi_deg
            self._send_steps(steps, positive)
            time.sleep(0.1)     # brief settle before re-reading encoder
            after = self.phi_deg
            if abs(after - before) < 0.1 and abs(target - after) > POSITION_TOL_DEG:
                print(f"  Phi stall at {self.phi_deg:.1f}° "
                      f"(attempt {attempt + 1}/{MAX_PHI_RETRIES}), retrying...")

        if abs(self.phi_deg - target) <= POSITION_TOL_DEG:
            print(f"  Phi reached {self.phi_deg:.1f}°")
        else:
            print(f"  Warning: phi stopped at {self.phi_deg:.1f}° "
                  f"(target {target:.1f}°) after {MAX_PHI_RETRIES} attempts")

    # ── Main scan loop ────────────────────────────────────────────────────────

    def run_scan(self):
        """
        Full scan pattern:
          - Motor 1 sweeps 0° ↔ 180° repeatedly.
          - Motor 2 advances PHI_STEP_DEG after every one-way sweep.
          - When phi reaches PHI_LIMIT_DEG, Motor 1 completes one final
            180° sweep then both motors hold position.
        """
        print("Homing to start position...")
        self.move_servo_to(0.0)
        time.sleep(0.5)
        self._reset_encoders()
        print("Encoders zeroed. Starting scan.\n")

        theta_targets = [THETA_SWEEP_DEG, 0.0]     # alternating sweep targets
        sweep_index = 0

        while True:
            target = theta_targets[sweep_index % 2]
            print(f"─── Sweep {sweep_index + 1}: "
                  f"theta → {target:.1f}°  |  phi = {self.phi_deg:.1f}°")

            # Motor 1 sweep
            self.move_servo_to(target)

            # Motor 2 advance
            self.move_phi_by(PHI_STEP_DEG)
            sweep_index += 1

            # Check stop condition after phi advance
            if self.phi_deg >= PHI_LIMIT_DEG:
                final_target = theta_targets[sweep_index % 2]
                print(f"\nPhi limit reached ({self.phi_deg:.1f}°). "
                      f"Completing final sweep to {final_target:.1f}°...")
                self.move_servo_to(final_target)
                print(f"\nScan complete. Holding position — "
                      f"theta={self.theta_deg:.1f}°, phi={self.phi_deg:.1f}°")
                break

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self):
        GPIO.remove_event_detect(ENC1_A)
        GPIO.remove_event_detect(ENC2_A)
        self._pwm.stop()
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.cleanup()


def main():
    controller = GantryController()
    try:
        controller.run_scan()
        print("Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        controller.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
