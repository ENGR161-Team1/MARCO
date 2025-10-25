from buildhat import MotorPair, Motor
from basehat import LineFinder

def main():
        # Initialize the motor pair on ports A and B
        motor_pair = MotorPair('A', 'B')
        line_finder_pin = 16
        #grovepi.pinMode(line_finder_pin, "INPUT")

        # Move the motor pair forward for 2 seconds at 50% speed
        # motor_pair.start_timed(50, 2000)
        linefinder = LineFinder(line_finder_pin)
        while True:
                sensor_value = linefinder.value
                print("Line Finder Sensor Value:", sensor_value)
                if sensor_value == 1:
                        # print("On Track")
                        motor_pair.start(-50, 50)  # Move forward
                else:
                        # print("Off Track")
                        motor_pair.start(50, -50)  # Move backward

if __name__ == "__main__":
    main()