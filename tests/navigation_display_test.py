"""
navigation_display_test.py

Mobility test with integrated Navigation3D and NavigationDisplay visualization.

This combines the safety ring obstacle avoidance from MotionController
with real-time navigation tracking and visual display.
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import asyncio
from basehat import IMUSensor
from systems.mobility_system import MotionController
from systems.navigation_system import Navigation3D
from ui.navigation_display import NavigationDisplay

# Motion controller (handles motors and safety ring)
motion = MotionController(
    front_motor="A",
    turn_motor="B",
    ultrasonic_pin=26,
    slowdown_distance=30.0,
    stopping_distance=15.0,
    forward_speed=20,
    forward_speed_slow=10
)

# Navigation
imu_sensor = IMUSensor()
navigator = Navigation3D(imu=imu_sensor, mode="degrees")

# Display
display = NavigationDisplay(
    width=800,
    height=800,
    scale=50.0,
    navigator=navigator
)


async def start_navigation():
    """Run continuous navigation updates with logging."""
    await navigator.run_continuous_update(
        update_interval=0.1,
        log_state=True,
        print_state=False  # Display handles visualization
    )


async def start_display():
    """Run the navigation display with live updates from navigator."""
    await display.run_continuous(update_interval=0.1)


async def main():
    """Run safety ring, navigation, and display concurrently."""
    await asyncio.gather(
        motion.start_safety_ring(),
        start_navigation(),
        start_display()
    )


if __name__ == "__main__":
    try:
        motion.start()
        asyncio.run(main())
    except KeyboardInterrupt:
        motion.stop()
        display.close()
        print(f"\nProgram terminated. Motors stopped.")
        if navigator.log:
            print(f"Logged {len(navigator.log)} navigation entries over "
                  f"{navigator.log[-1]['timestamp']:.2f} seconds.")
