#!/home/mie_g28/venv/bin/python3
"""
test_motors.py
──────────────
Standalone gantry motor test — NO ROS 2 required.

Run directly on the Pi to verify Motor 1 (servo, theta) and
Motor 2 (stepper, phi) are wired and working before launching
any ROS 2 nodes.

Usage:
  python3 tests/test_motors.py

What it does:
  1. Motor 1 (theta) — servo sweeps 0° → 120° → 0°.
  2. Motor 2 (phi)   — stepper moves forward 16000 steps, stops when
                       encoder detects a stall (motor has hit its limit).

Troubleshooting:
  Servo doesn't move      → Check 24V power, PWM signal on BCM 13 (Pin 33).
  Servo jitters           → Check PWM signal quality, ensure good ground connection.
  Stepper doesn't move    → Check EN pin is LOW, VMOT connected,
                            check TB6600 DIP switches.
  Stepper wrong direction → Swap one coil pair (A+↔A− or B+↔B− on TB6600).
  Encoder not counting    → Check ENC2_A/B wiring on BCM 14 and BCM 19.
"""

import RPi.GPIO as GPIO
import time

# ── Pin assignments — must match motor_controller.py ──────────────────────────
SERVO_PIN = 13                          # Motor 1 — theta (Z rotation)
M2_STEP = 23;  M2_DIR = 24;  M2_EN = 25  # Motor 2 — phi (arc position)
ENC2_A = 14;   ENC2_B = 19             # Encoder 2 — Motor 2 (phi)

# ── Servo constants ───────────────────────────────────────────────────────────
SERVO_FREQ          = 50
SERVO_MIN_PULSE     = 0.5
SERVO_MAX_PULSE     = 2.5
SERVO_MAX_ANGLE     = 300.0
SERVO_SWEEP_TIME    = 8.0       # seconds for a full 180° sweep
SERVO_INCREMENT_DEG = 2.0       # degrees per increment

# ── Stepper constants ─────────────────────────────────────────────────────────
STEP_DELAY        = 0.002
M2_STEPS          = 16000
CHUNK_SIZE        = 200         # steps per encoder-check batch
ENC2_COUNT_LIMIT  = 1200        # encoder counts at which Motor 2 stops — TUNE THIS
PAUSE_S           = 1.0

# ── Encoder state ─────────────────────────────────────────────────────────────
enc2_count = 0


def enc2_cb(channel):
    global enc2_count
    a = GPIO.input(ENC2_A)
    b = GPIO.input(ENC2_B)
    enc2_count += 1 if a != b else -1


def angle_to_duty(angle):
    angle = max(0.0, min(SERVO_MAX_ANGLE, angle))
    pulse_ms = SERVO_MIN_PULSE + (angle / SERVO_MAX_ANGLE) * \
               (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
    return (pulse_ms / (1000.0 / SERVO_FREQ)) * 100.0


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    for pin in [M2_STEP, M2_DIR, M2_EN]:
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(M2_EN, GPIO.LOW)
    for pin in [ENC2_A, ENC2_B]:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENC2_A, GPIO.BOTH, callback=enc2_cb)
    print("GPIO initialised. Servo, stepper driver, and encoder enabled.")


def servo_move(angle, label=""):
    print(f"  {label}: moving to {angle}°...")
    start = pwm_current_angle[0]
    delta = angle - start
    steps = max(1, int(round(abs(delta) / SERVO_INCREMENT_DEG)))
    delay = (abs(delta) / 180.0) * SERVO_SWEEP_TIME / steps
    for i in range(1, steps + 1):
        intermediate = start + (delta * i / steps)
        pwm.ChangeDutyCycle(angle_to_duty(intermediate))
        time.sleep(delay)
    pwm_current_angle[0] = angle
    print(f"  {label}: done.")


def stepper_move_encoder(total_steps, label=""):
    """Move stepper forward in chunks, stopping when encoder hits ENC2_COUNT_LIMIT."""
    print(f"  {label}: moving forward up to {total_steps} steps "
          f"(encoder limit: {ENC2_COUNT_LIMIT} counts)...")
    GPIO.output(M2_DIR, GPIO.HIGH)
    remaining = total_steps
    while remaining > 0:
        if enc2_count >= ENC2_COUNT_LIMIT:
            print(f"  {label}: encoder limit reached ({enc2_count} counts) — stopping.")
            break
        batch = min(CHUNK_SIZE, remaining)
        for _ in range(batch):
            GPIO.output(M2_STEP, GPIO.HIGH)
            time.sleep(STEP_DELAY)
            GPIO.output(M2_STEP, GPIO.LOW)
            time.sleep(STEP_DELAY)
        remaining -= batch
    print(f"  {label}: done. Final encoder count: {enc2_count}")


def main():
    global pwm, pwm_current_angle
    pwm_current_angle = [0.0]   # tracks current servo angle for incremental moves
    print("═" * 60)
    print("  ScanToCAD — Gantry Motor Test")
    print("═" * 60)

    try:
        setup()
        pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        pwm.start(angle_to_duty(0.0))
        time.sleep(0.5)

        print("\n── Test: Motor 1 (theta — servo) ────────────────────────────")
        print("  Expected: servo sweeps 0° → 120° → 0°.")
        servo_move(145.0, "Motor 1 (theta)")
        time.sleep(PAUSE_S)
        servo_move(  0.0, "Motor 1 (theta)")
        time.sleep(PAUSE_S)
        print("\n✓ Motor 1 test complete.")

        print("\n── Test: Motor 2 (phi — stepper arc position) ───────────────")
        print(f"  Expected: head moves forward up to {M2_STEPS} steps, stops on stall.")
        stepper_move_encoder(M2_STEPS, "Motor 2 (phi)")
        print("\n✓ Motor 2 test complete.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        try:
            pwm.stop()
            del pwm
        except Exception:
            pass
        GPIO.remove_event_detect(ENC2_A)
        GPIO.output(M2_EN, GPIO.HIGH)
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    main()
