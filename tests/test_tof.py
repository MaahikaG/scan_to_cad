#!/home/mie_g28/venv/bin/python3
"""
test_tof.py
───────────
Standalone VL53L0X TOF sensor test — NO ROS 2 required.

Prints live distance readings at ~10 Hz.

Usage:
  python3 tests/test_tof.py

Install library if needed:
  pip install adafruit-blinka adafruit-circuitpython-vl53l0x
"""

import board
import busio
import adafruit_vl53l0x
import time


def main():
    print("Initialising VL53L0X...")
    i2c = busio.I2C(board.SCL, board.SDA)
    tof = adafruit_vl53l0x.VL53L0X(i2c)
    tof.measurement_timing_budget = 33000  # BETTER mode ~66 ms
    print("Ready. Hold objects at varying distances. Ctrl+C to exit.\n")
    print(f"{'#':>6}  {'mm':>10}  {'metres':>10}")
    print("-" * 32)

    n = 0
    try:
        while True:
            d = tof.range
            n += 1
            if d < 8190:
                print(f"{n:>6}  {d:>10.1f}  {d/1000:>10.4f}")
            else:
                print(f"{n:>6}  {'OUT OF RANGE':>10}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        i2c.deinit()
        print("Sensor stopped.")


if __name__ == '__main__':
    main()