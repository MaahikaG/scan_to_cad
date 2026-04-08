#!/home/mie_g28/venv/bin/python3
"""
motor_controller.py
───────────────────
Standalone controller for all four motors. No ROS.

SCAN PATTERN:
  Gantry Motor 1 (servo, theta) sweeps 0° → 170° → 0° → ... in ~45° steps.
  At each ~45° stop, a full pan-tilt sweep is performed (Motor A × Motor B).
  After a complete one-way gantry sweep, Gantry Motor 2 (stepper, phi)
  advances by PHI_STEP_STEPS. Scan ends when phi reaches PHI_LIMIT_STEPS.

PAN-TILT SWEEP (at each gantry theta stop):
  Motor A (pan) steps through 0 → PT_A_STEPS in PT_A_INC increments.
  At each Motor A position, Motor B (tilt) sweeps 0 → PT_B_STEPS and back.
  PT_PAUSE_S is waited at each stop so the TOF sensor can capture a reading.

GPIO (BCM numbering):
  Gantry Motor 1 (servo, θ):   PWM=13
  Gantry Motor 2 (stepper, φ): STEP=23  DIR=24  EN=25
  Encoder 1 (gantry θ):        A=5   B=6
  Pan-tilt Motor A (pan):      STEP=16  DIR=20  EN=21
  Pan-tilt Motor B (tilt):     STEP=12  DIR=7   EN=8
"""

import RPi.GPIO as GPIO
import time
import threading

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
THETA_FORWARD_STOPS = [0.0, 100.0, 135.0, 170.0]   # forward sweep stop angles
THETA_RETURN_STOPS  = [170.0, 135.0, 100.0, 0.0]   # return sweep stop angles
PHI_STEP_STEPS      = 2909          # stepper steps per gantry sweep
                                    # = round(16000 * 30/165)
PHI_LIMIT_STEPS     = 16000         # total phi steps before scan ends

# ── Pan-tilt sweep parameters ─────────────────────────────────────────────────
PT_STEP_DELAY       = 0.001         # seconds per pulse edge
PT_STEP_DEG         = 1.8           # degrees per full step (NEMA 8, full step)
PT_A_STEPS          = 400           # full range of Motor A (pan)  — TUNE THIS
PT_B_STEPS          = 400           # full range of Motor B (tilt) — TUNE THIS
PT_A_INC            = 100           # Motor A steps per increment
PT_B_INC            = 100           # Motor B steps per increment
PT_PAUSE_S          = 0.15          # pause at each stop for TOF reading

# ── Servo control parameters ──────────────────────────────────────────────────
SERVO_SETTLE_S      = 2.0           # time to wait after sending servo command

# ── Encoder debounce ──────────────────────────────────────────────────────────
DEBOUNCE_MS         = 3.0


class GantryController:
    def __init__(self):
        self._lock           = threading.Lock()
        self._enc1_count     = 0
        self._enc1_last_ms   = 0.0
        self._phi_steps_sent = 0
        self._pt_a_steps     = 0
        self._pt_b_steps     = 0

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
        self._pwm.ChangeDutyCycle(self._angle_to_duty(target_deg))
        time.sleep(SERVO_SETTLE_S)
        print(f"  Gantry θ → {target_deg:.1f}° (encoder reads {self.theta_deg:.1f}°)")

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
        Pauses at every stop so the TOF sensor can capture a reading.
        """
        a_positions = list(range(0, PT_A_STEPS, PT_A_INC)) + [PT_A_STEPS]

        for a_target in a_positions:
            a_delta = a_target - self._pt_a_steps
            if a_delta != 0:
                self._step(PA_STEP, PA_DIR, abs(a_delta),
                           a_delta > 0, PT_STEP_DELAY)
                self._pt_a_steps = a_target

            b_positions = list(range(0, PT_B_STEPS, PT_B_INC)) + [PT_B_STEPS]
            for b_target in b_positions:
                b_delta = b_target - self._pt_b_steps
                if b_delta != 0:
                    self._step(PB_STEP, PB_DIR, abs(b_delta),
                               b_delta > 0, PT_STEP_DELAY)
                    self._pt_b_steps = b_target
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

    # ── Main scan loop ────────────────────────────────────────────────────────

    def run_scan(self):
        print("Homing servo to 0°...")
        self.move_servo_to(0.0)
        time.sleep(0.5)
        with self._lock:
            self._enc1_count = 0
        print("Encoder zeroed. Starting scan.\n")

        sweep_stops = [THETA_FORWARD_STOPS, THETA_RETURN_STOPS]
        sweep_index = 0

        while True:
            stops = sweep_stops[sweep_index % 2]
            print(f"═══ Sweep {sweep_index + 1} │ "
                  f"{'→'.join(f'{p}°' for p in stops)} │ "
                  f"φ steps={self._phi_steps_sent}")

            for pos in stops:
                self.move_servo_to(pos)
                self._pan_tilt_sweep(pos)

            if self._phi_steps_sent >= PHI_LIMIT_STEPS:
                print("\nPhi limit reached. Scan complete.")
                break

            self.move_phi_steps(PHI_STEP_STEPS)
            sweep_index += 1

            if self._phi_steps_sent >= PHI_LIMIT_STEPS:
                stops = sweep_stops[sweep_index % 2]
                print(f"\nCompleting final sweep...")
                for pos in stops:
                    self.move_servo_to(pos)
                    self._pan_tilt_sweep(pos)
                print("\nScan complete.")
                break

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self):
        GPIO.remove_event_detect(ENC1_A)
        self._pwm.stop()
        del self._pwm
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.output(PA_EN, GPIO.HIGH)
        GPIO.output(PB_EN, GPIO.HIGH)
        GPIO.cleanup()


def main():
    controller = GantryController()
    try:
        controller.run_scan()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        controller.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
