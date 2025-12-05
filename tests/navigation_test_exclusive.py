"""
navigation_test_exclusive.py

Navigation-only test without mobility.

Tests Navigation3D position tracking using the IMU sensor
without any motor control or safety ring.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from basehat import IMUSensor
from systems.navigation_system import Navigation3D

# Navigation
imu_sensor = IMUSensor()
navigator = Navigation3D(imu=imu_sensor, mode="degrees")


async def main():
    """Calibrate IMU and run navigation updates."""
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=True,
        calibrate=True
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print(f"\nProgram terminated.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
