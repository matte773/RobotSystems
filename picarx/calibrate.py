import time
from picarx_improved import Picarx

def calibrate_steering():
    # Initialize the picar controller
    picar = Picarx()

    # Set initial calibration angle (you may adjust this based on your observations)
    initial_calibration_angle = 0

    # Command the system to drive forward at zero steering angle
    picar.set_dir_servo_angle(initial_calibration_angle)
    picar.forward(50)  # Adjust speed as needed
    time.sleep(5)  # Adjust duration as needed
    picar.stop()

    # Measure how far left or right the car pulls during calibration
    #deviation = picar.measure_deviation()

    # Modify the steering calibration angle in picarx_improved.py based on the deviation
    # new_calibration_angle = initial_calibration_angle - deviation
    # YourPicarController.CALIBRATION_ANGLE = new_calibration_angle

    # Print the updated calibration angle
    #print(f"Updated Calibration Angle: {new_calibration_angle}")

if __name__ == "__main__":
    calibrate_steering()
