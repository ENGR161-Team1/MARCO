"""
macro_navigation.py

Autonomous line-guidance navigation for MACRO (based on Section 2.3 Navigation System).

This file contains:
 - Navigator: class implementing PID line following with gap-handling and line-type detection.
 - Hardware abstraction functions (placeholders) to be implemented with actual Pi/BuildHat/Grove APIs.
 - Example main() showing how to run the navigation loop.

Note: You MUST replace the hardware abstraction placeholders with your project's sensor/motor API calls.
"""

import time
import math
from collections import deque

# ---------------------------
# Configuration / Constants
# ---------------------------

# Units: inches / cm as noted; convert as necessary.
INCH_TO_CM = 2.54

# Minimum radius of curvature for guideline centerline (RFP): 2.0 inches
MIN_RADIUS_IN = 2.0
MIN_RADIUS_CM = MIN_RADIUS_IN * INCH_TO_CM

# Robot physical params (tweak for your robot)
WHEEL_BASE_CM = 12.0  # distance between left and right wheels (example)
WHEEL_RADIUS_CM = 3.0  # wheel radius (example)

# Max linear speed (cm/s) - keep below allowed project speed or your hardware limits
MAX_LINEAR_SPEED = 25.0  # example; obey RFP limits elsewhere (15-30 cm/s subtrack)
# Convert curvature constraint to max angular velocity
# v = linear speed, omega = v / R  => omega_max = MAX_LINEAR_SPEED / MIN_RADIUS_CM
OMEGA_MAX = MAX_LINEAR_SPEED / max(MIN_RADIUS_CM, 0.1)

# PID gains for steering (tune on hardware)
KP = 0.9
KI = 0.002
KD = 0.02

# Line sensor specifics
NUM_SENSORS = 5          # e.g., 5-element line sensor array
SENSOR_THRESHOLD = 0.5   # normalized threshold for line detection (0..1); adjust per sensor
SENSOR_POSITIONS = [-2, -1, 0, 1, 2]  # positions for sensors across wheelbase (arbitrary units)

# Gap handling parameters
GAP_MAX_REACQUIRE_TIME = 1.2  # seconds to attempt reacquire before stopping
GAP_SWEEP_ANGLE_DEG = 20      # degrees of sweep when attempting to find line
GAP_SWEEP_STEP = 5            # degrees per step
REACQUIRE_MAX_DISTANCE_CM = 40.0  # maximum dead-reckon forward distance while searching

# Line-type detection window
LINE_WINDOW_DISTANCE_CM = 50.0  # window length used to classify dotted/broken/continuous
LINE_WINDOW_SAMPLES = 25

# Control loop timing
LOOP_DT = 0.04  # 40 ms control loop

# ---------------------------
# Hardware abstraction layer (PLACEHOLDERS)
# ---------------------------

def read_line_sensors():
    """
    Read line-finder sensor array and return a list of NUM_SENSORS float values normalized [0..1].
    High value -> dark line (black); Low -> background.

    TODO: Replace with actual sensor API. The ordering should match SENSOR_POSITIONS left-to-right.
    Example return for 5 sensors: [0.1, 0.3, 0.9, 0.2, 0.05]
    """
    # Placeholder implementation (for testing without hardware)
    raise NotImplementedError("Implement read_line_sensors() using your Grove/line-finder API.")

def set_motor_speeds(left_cm_s, right_cm_s):
    """
    Set motor speeds in linear cm/s for left and right wheels.

    TODO: Replace with actual motor control API (BuildHat or motor HAT).
    """
    raise NotImplementedError("Implement set_motor_speeds() for your motor driver.")

def read_encoders():
    """
    Return (left_ticks, right_ticks) or (left_cm, right_cm) traveled since last call.
    Implementation-dependent. For dead-reckoning, we prefer distance in cm since last read.

    TODO: Replace with actual encoder reading.
    """
    raise NotImplementedError("Implement read_encoders() for your hardware (return distance in cm).")

def read_magnetometer():
    """
    Optional: read magnetometer or digital magnet sensor to detect buried beacon.
    Return strength/boolean indicating a beacon is detected.

    TODO: Replace with actual magnetometer API if you plan to use branch detection via buried beacons.
    """
    return None  # default: no magnetometer

def stop_motors():
    """Convenience to stop both motors."""
    set_motor_speeds(0.0, 0.0)

# ---------------------------
# Helper math functions
# ---------------------------

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# ---------------------------
# Navigator class
# ---------------------------

class Navigator:
    def __init__(self):
        # PID accumulators
        self.integral = 0.0
        self.prev_error = 0.0

        # For line-type detection (sliding window of detections)
        self.line_presence_window = deque(maxlen=LINE_WINDOW_SAMPLES)
        self.distance_since_window_start = 0.0

        # Encoder bookkeeping for dead-reckoning
        self._last_encoder_read = (0.0, 0.0)  # left_cm, right_cm (initialize on first reading)

    def sensor_reading_to_position(self, sensor_vals):
        """
        Convert an array of sensor readings to a lateral error (negative = left, positive = right).
        We'll use a weighted average (center-of-mass) where darker values (higher) bias the position.
        If no sensor sees the line (all below threshold), return None.
        """
        weights = []
        total = 0.0
        weighted_sum = 0.0
        for val, pos in zip(sensor_vals, SENSOR_POSITIONS):
            # consider reading only above threshold to avoid noise
            if val >= SENSOR_THRESHOLD:
                w = val  # weight by intensity
                weights.append((pos, w))
                total += w
                weighted_sum += pos * w

        if total == 0.0:
            return None  # no line detected
        return weighted_sum / total

    def pid_steer(self, error, dt):
        """Simple PID controller returning an angular command (omega) in rad/s (signed)."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # Compute steering command (in arbitrary units) and convert to omega via a gain factor
        steering = KP * error + KI * self.integral + KD * derivative

        # Map steering to angular velocity and clamp by OMEGA_MAX
        # Here steering is normalized relative to sensor positions; choose scale factor
        STEERING_SCALE = 1.0  # tune this
        omega = clamp(steering * STEERING_SCALE, -OMEGA_MAX, OMEGA_MAX)
        return omega

    def omega_to_wheel_speeds(self, v_cm_s, omega):
        """
        Convert linear velocity (cm/s) and angular velocity (rad/s) to left and right wheel speeds (cm/s).
        Using differential drive kinematics:
            v = (v_r + v_l) / 2
            omega = (v_r - v_l) / L  => v_r = v + omega*L/2 ; v_l = v - omega*L/2
        """
        L = WHEEL_BASE_CM
        v_r = v_cm_s + (omega * L / 2.0)
        v_l = v_cm_s - (omega * L / 2.0)
        # clamp to motor limits
        v_r = clamp(v_r, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        v_l = clamp(v_l, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
        return v_l, v_r

    def update_line_window(self, sees_line, distance_increment):
        """
        Maintain a window of booleans indicating whether line was present in recent samples,
        along with the cumulative distance over which that window spans (approx).
        """
        self.line_presence_window.append(1 if sees_line else 0)
        self.distance_since_window_start += distance_increment
        # Reset distance if window size just wrapped
        if len(self.line_presence_window) == self.line_presence_window.maxlen:
            # recompute approximate distance per sample:
            # We don't have exact timestamps per sample; using approximate average
            # Keep the value; if measurement is used, keep distance sliding by estimate
            pass

    def classify_line_type(self):
        """
        Use the presence window to guess the type:
         - continuous: high proportion of '1's
         - dotted: periodic 1s with spacing ~2in centers -> moderate ratio with pattern
         - broken: variable long breaks -> low proportion but irregular
        This is heuristic and for diagnostics only.
        """
        if len(self.line_presence_window) == 0:
            return "unknown"
        ratio = sum(self.line_presence_window) / len(self.line_presence_window)
        if ratio > 0.85:
            return "continuous"
        if 0.4 <= ratio <= 0.85:
            return "dotted/broken"
        return "mostly broken"

    def follow_line(self, desired_speed_cm_s=15.0, run_time_s=9999.0):
        """
        Main line-following loop. Attempts to follow line indefinitely (or until run_time_s).
        Returns when stopped (e.g., cannot reacquire line).
        """
        t_start = time.time()
        last_time = t_start
        distance_traveled_since_last = 0.0

        # Initialize encoders baseline if available
        try:
            l_enc, r_enc = read_encoders()
            self._last_encoder_read = (l_enc, r_enc)
        except Exception:
            # hardware may not support; proceed anyway
            pass

        while True:
            t_now = time.time()
            dt = t_now - last_time
            if dt < LOOP_DT:
                time.sleep(LOOP_DT - dt)
                t_now = time.time()
                dt = t_now - last_time
            last_time = t_now

            # Read sensors
            try:
                sensor_vals = read_line_sensors()
            except NotImplementedError:
                print("read_line_sensors() not implemented. Exiting follow_line.")
                break

            # Compute lateral error from sensor array
            pos = self.sensor_reading_to_position(sensor_vals)
            sees_line = pos is not None

            # Estimate forward distance since last loop via encoders if available
            distance_increment = 0.0
            try:
                l_enc, r_enc = read_encoders()
                # assuming read_encoders returns cumulative distance in cm
                dl = l_enc - self._last_encoder_read[0]
                dr = r_enc - self._last_encoder_read[1]
                self._last_encoder_read = (l_enc, r_enc)
                distance_increment = (dl + dr) / 2.0
            except Exception:
                # fallback estimate: v*dt
                distance_increment = desired_speed_cm_s * dt

            # Update line presence history
            self.update_line_window(sees_line, distance_increment)

            if sees_line:
                # Reset gap search state if any
                # Compute an error: if pos==0 -> centered; pos <0 left; >0 right
                error = pos  # using sensor positions as error
                omega = self.pid_steer(error, dt)
                # Convert omega to wheel speeds while respecting curvature constraint
                v = min(desired_speed_cm_s, MAX_LINEAR_SPEED)
                left_speed, right_speed = self.omega_to_wheel_speeds(v, omega)
                set_motor_speeds(left_speed, right_speed)
            else:
                # Handle gap: attempt reacquire
                print("[NAV] Line lost: attempting reacquire")
                success = self.attempt_reacquire(desired_speed_cm_s)
                if not success:
                    print("[NAV] Reacquire failed after timeout - stopping")
                    stop_motors()
                    return False  # failed to maintain line
                else:
                    # reacquired, continue loop
                    continue

            # Optionally classify and log line type every window
            if len(self.line_presence_window) == self.line_presence_window.maxlen:
                classification = self.classify_line_type()
                # For diagnostics you could print occasionally
                # print(f"[NAV] Line type estimate over window: {classification}")
                # reset window distance if desired
                # self.line_presence_window.clear()

            # Check run_time limit
            if (t_now - t_start) > run_time_s:
                print("[NAV] Completed requested run_time; stopping navigation")
                stop_motors()
                return True

    def attempt_reacquire(self, base_speed_cm_s):
        """
        Strategy when line is lost:
        1) Continue forward at a reduced speed for a short 'dead-reckon' distance, attempting to resample.
        2) If still no line, perform lateral sweep (rotate in place / differential drive steer) alternating left/right sweeping
           within a limited angle, re-sampling sensors at each step.
        3) If line found, resume normal following. If not found within GAP_MAX_REACQUIRE_TIME or REACQUIRE_MAX_DISTANCE_CM -> fail.
        """
        t0 = time.time()
        # 1) Dead-reckon forward a little while scanning sensors
        DEAD_RECKON_SPEED = base_speed_cm_s * 0.4
        DEAD_RECKON_TIMEOUT = min(GAP_MAX_REACQUIRE_TIME * 0.6, 0.6)  # seconds
        print(f"[NAV] Dead-reckoning forward for up to {DEAD_RECKON_TIMEOUT:.2f}s at {DEAD_RECKON_SPEED:.1f} cm/s")
        t_dead_start = time.time()
        dist_travelled = 0.0
        while time.time() - t_dead_start < DEAD_RECKON_TIMEOUT and dist_travelled < REACQUIRE_MAX_DISTANCE_CM:
            # drive forward slowly
            v = DEAD_RECKON_SPEED
            set_motor_speeds(v, v)
            time.sleep(0.05)
            # check sensors
            try:
                sensor_vals = read_line_sensors()
            except NotImplementedError:
                break
            pos = self.sensor_reading_to_position(sensor_vals)
            if pos is not None:
                print("[NAV] Reacquired line during dead-reckon")
                return True
            # update distance estimate
            try:
                l_enc, r_enc = read_encoders()
                dl = l_enc - self._last_encoder_read[0]
                dr = r_enc - self._last_encoder_read[1]
                self._last_encoder_read = (l_enc, r_enc)
                dist_travelled += (dl + dr) / 2.0
            except Exception:
                dist_travelled += v * 0.05

        # 2) Stop forward motion
        stop_motors()

        # 3) Perform sweep rotations left and right
        # We'll perform small angular steps and check sensors after each.
        SWEEP_STEPS = int(GAP_SWEEP_ANGLE_DEG // GAP_SWEEP_STEP)
        step_deg = GAP_SWEEP_STEP
        step_rad = math.radians(step_deg)
        sweep_start_time = time.time()
        sweep_dir_sequence = []
        # generate sequence: 0, +1, -1, +2, -2 ... in steps of 'step_deg'
        for k in range(SWEEP_STEPS):
            if k == 0:
                sweep_dir_sequence.append(0)
            else:
                sweep_dir_sequence.append(k)
                sweep_dir_sequence.append(-k)

        for idx, step_mult in enumerate(sweep_dir_sequence):
            if time.time() - sweep_start_time > GAP_MAX_REACQUIRE_TIME:
                break
            angle = step_mult * step_rad
            # Convert desired angle to differential wheel speeds for a short duration
            # Approx angular motion model: omega = v_angular ; wheel speeds v_l = -omega*L/2, v_r = +omega*L/2 for in-place spin
            omega = clamp((angle / 0.2), -OMEGA_MAX, OMEGA_MAX)  # aim to achieve angle over ~0.2 s
            v_l = -omega * WHEEL_BASE_CM / 2.0
            v_r = omega * WHEEL_BASE_CM / 2.0
            # clamp speeds
            v_l = clamp(v_l, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            v_r = clamp(v_r, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            # execute the small rotation
            set_motor_speeds(v_l, v_r)
            # brief sleep to effect rotation
            time.sleep(0.12)
            # stop and sample sensors
            stop_motors()
            time.sleep(0.03)
            try:
                sensor_vals = read_line_sensors()
            except NotImplementedError:
                break
            pos = self.sensor_reading_to_position(sensor_vals)
            if pos is not None:
                print(f"[NAV] Reacquired line during sweep at step {step_mult}")
                return True

        # final attempt: small forward/backward wiggle
        for _ in range(3):
            # short forward
            set_motor_speeds(base_speed_cm_s * 0.2, base_speed_cm_s * 0.2)
            time.sleep(0.08)
            stop_motors()
            time.sleep(0.03)
            try:
                sensor_vals = read_line_sensors()
            except NotImplementedError:
                break
            if self.sensor_reading_to_position(sensor_vals) is not None:
                print("[NAV] Reacquired line during wiggle")
                return True

        # timed out without reacquire
        return False

# ---------------------------
# Example main usage
# ---------------------------

def main():
    """
    Example top-level run. Replace the hardware placeholder implementations before running.
    """
    navigator = Navigator()

    # defaults / example parameters; in real demo, choose based on mission phase
    desired_speed = 18.0  # cm/s (tune per hardware)
    run_time = 300.0      # seconds (example)

    print("[MAIN] Starting line-following navigation. Press Ctrl-C to abort.")
    try:
        success = navigator.follow_line(desired_speed_cm_s=desired_speed, run_time_s=run_time)
        if success:
            print("[MAIN] Navigation completed successfully.")
        else:
            print("[MAIN] Navigation aborted due to failed reacquire or error.")
    except KeyboardInterrupt:
        print("[MAIN] Keyboard interrupt received - stopping.")
        stop_motors()

if __name__ == "__main__":
    main()