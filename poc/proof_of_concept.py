from buildhat import MotorPair, Motor

def main():
    # Initialize the motor pair on ports A and B
    motor_pair = MotorPair('A', 'B')
    
    # Move the motor pair forward for 2 seconds at 50% speed
    motor_pair.start_timed(50, 2000)

if __name__ == "__main__":
    main()