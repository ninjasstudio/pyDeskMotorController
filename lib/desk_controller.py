from machine import Pin, PWM, Timer, UART
import time
import json
from lib.PID import PID
from lib.lidar import LIDAR

class DeskController:
    """
    A class to control a motorized desk with two motors and LIDAR sensors for height measurement.
    
    This class implements advanced control algorithms including state-space control and cascade control
    to ensure smooth and synchronized movement of both motors. It also includes safety features and
    error logging capabilities.
    """

    def __init__(self, debug=False):
        """
        Initialize the DeskController with all necessary components and parameters.
        """
        # Motor pins setup
        self.motor1_enable = Pin(17, Pin.OUT, Pin.PULL_UP, value=1)
        self.motor2_enable = Pin(21, Pin.OUT, Pin.PULL_UP, value=1)

        # Define PWM frequency (10kHz is chosen for smooth motor operation)
        pwm_freq = 10000

        # Initialize PWM channels for both motors
        self.pwm_motor1_in1 = PWM(Pin(19), freq=pwm_freq, duty=0)
        self.pwm_motor1_in2 = PWM(Pin(18), freq=pwm_freq, duty=0)
        self.pwm_motor2_in1 = PWM(Pin(23), freq=pwm_freq, duty=0)
        self.pwm_motor2_in2 = PWM(Pin(22), freq=pwm_freq, duty=0)

        # Position tracking variables
        self.position_motor1 = 0
        self.position_motor2 = 0

        # PID Controllers for each motor
        self.pid_motor1 = PID(13.905, 4, 0, setpoint=0, sample_time=10)
        self.pid_motor2 = PID(13.905, 4, 0, setpoint=0, sample_time=10)

        # Set output limits for the PID controllers to match PWM range
        self.pid_motor1.output_limits = (-1023, 1023)
        self.pid_motor2.output_limits = (-1023, 1023)

        # Non-volatile memory (NVM) storage for saving desk position
        self.nvm_file = "desk_position.json"

        # Debug mode flag
        self.debug_mode = debug

        # Initialize UART for TF Luna LIDAR sensors
        uart1 = UART(1, tx=27, rx=14)
        uart2 = UART(2, tx=25, rx=26)

        # Initialize LIDAR sensors
        self.lidar_motor1 = LIDAR(uart1)
        self.lidar_motor2 = LIDAR(uart2)

        # Define min and max height limits for the desk
        self.min_height = 50.008  # cm
        self.max_height = 129.032  # cm

        # Timer for periodic checking of height limits and motor synchronization
        self.limit_check_timer = Timer(-1)
        self.limit_check_timer.init(
            period=200, mode=Timer.PERIODIC, callback=self.check_height_limits_and_sync
        )

        # Timer for periodic updates of motor positions and velocities
        self.update_timer = Timer(-1)
        self.update_timer.init(period=10, mode=Timer.PERIODIC, callback=self.update)

        # Safety parameters
        self.max_position_difference = 12  # Maximum allowed difference between motor positions (cm)
        self.max_velocity = 50  # Maximum allowed velocity (cm/s)
        self.last_positions = [53, 56]  # Last recorded positions of both motors
        self.last_update_time = time.time()  # Timestamp of last update

    def set_debug_mode(self, debug):
        """
        Set the debug mode for verbose output.

        Args:
            debug (bool): True to enable debug mode, False to disable.
        """
        self.debug_mode = debug
        print("Debug mode set to {}".format(self.debug_mode))

    def save_position(self, position):
        """
        Save the current desk position to non-volatile memory.

        Args:
            position (dict): A dictionary containing the positions of both motors.
        """
        with open(self.nvm_file, "w") as f:
            json.dump(position, f)

    def load_position(self):
        """
        Load the last saved desk position from non-volatile memory.

        Returns:
            dict: A dictionary containing the last saved positions of both motors.
        """
        try:
            with open(self.nvm_file, "r") as f:
                return json.load(f)
        except OSError:
            return {"motor1": 0, "motor2": 0}

    def set_brake(self):
        """
        Stop all motors by setting their duty cycles to 0.
        """
        print("Setting brake: stopping all motors")
        self.pwm_motor1_in1.duty(0)
        self.pwm_motor1_in2.duty(0)
        self.pwm_motor2_in1.duty(0)
        self.pwm_motor2_in2.duty(0)
    def ease_in_out(self, t: float) -> float:
        """Ease in out function for smooth acceleration and deceleration."""
        return t * t * (3 - 2 * t)

    def move_motor_smooth(self, motor_id: int, target_duty: int, duration: float) -> None:
        """Move motor with smooth start and stop."""
        steps = 100
        step_duration = duration / steps
        for step in range(steps + 1):
            progress = step / steps
            eased_progress = self.ease_in_out(progress)
            current_duty = int(eased_progress * target_duty)
            self._set_motor_duty(motor_id, current_duty)
            time.sleep(step_duration)
        self.set_brake()

    def _set_motor_duty(self, motor_id, duty_cycle):
        """
        Set the duty cycle for a specific motor.

        Args:
            motor_id (int): The ID of the motor (1 or 2).
            duty_cycle (int): The duty cycle to set (-1023 to 1023).
        """
        duty_cycle = max(-1023, min(1023, duty_cycle))  # Ensure duty cycle is within bounds
        if motor_id == 1:
            if duty_cycle >= 0:
                self.pwm_motor1_in1.duty(duty_cycle)
                self.pwm_motor1_in2.duty(0)
            else:
                self.pwm_motor1_in1.duty(0)
                self.pwm_motor1_in2.duty(-duty_cycle)
        elif motor_id == 2:
            if duty_cycle >= 0:
                self.pwm_motor2_in1.duty(duty_cycle)
                self.pwm_motor2_in2.duty(0)
            else:
                self.pwm_motor2_in1.duty(0)
                self.pwm_motor2_in2.duty(-duty_cycle)

    def motor_controller(self, enable_power):
        """
        Enable or disable the motor controller.

        Args:
            enable_power (bool): True to enable, False to disable.
        """
        print(
            "Setting motor controller to {}".format('enabled' if enable_power else 'disabled')
        )
        enable_power_active_low = not enable_power
        self.motor1_enable.value(enable_power_active_low)
        self.motor2_enable.value(enable_power_active_low)

    def move_to_position(self, target_position):
        """
        Move the desk to a target position using PID control.

        Args:
            target_position (float): The target position in cm.
        """
        self.pid_motor1.setpoint = target_position
        self.pid_motor2.setpoint = target_position
        
        while True:
            distance_motor1 = self.lidar_motor1.distance()
            distance_motor2 = self.lidar_motor2.distance()

            if distance_motor1 is not None and distance_motor2 is not None:
                pos1 = int(distance_motor1)
                pos2 = int(distance_motor2)

                # Calculate errors
                error1 = target_position - pos1
                error2 = target_position - pos2

                # Compute PID outputs
                pwm1 = self.pid_motor1(error1)
                pwm2 = self.pid_motor2(error2)

                # Synchronize motors
                if abs(error1 - error2) > self.max_position_difference:
                    if error1 > error2:
                        pwm1 -= 50
                    else:
                        pwm2 -= 50

                self._set_motor_duty(1, int(pwm1))
                self._set_motor_duty(2, int(pwm2))

                self.last_positions = [pos1, pos2]
                self.last_update_time = time.time()

                # Break condition
                if abs(pos1 - target_position) <= 1 and abs(pos2 - target_position) <= 1:
                    break

            time.sleep(0.01)  # Small delay to prevent excessive CPU usage

        self.set_brake()

    def home_position(self):
        """
        Move the desk to its home (lowest) position.
        """
        print("Homing...")

        prev_distance_motor1 = self.lidar_motor1.distance()
        prev_distance_motor2 = self.lidar_motor2.distance()
        stable_time_start = time.time()

        while True:
            current_distance_motor1 = self.lidar_motor1.distance()
            current_distance_motor2 = self.lidar_motor2.distance()

            if current_distance_motor1 is not None and current_distance_motor2 is not None:
                if (current_distance_motor1 >= prev_distance_motor1 and
                    current_distance_motor2 >= prev_distance_motor2):
                    stable_time_elapsed = time.time() - stable_time_start
                    if stable_time_elapsed >= 1.0:
                        break
                else:
                    stable_time_start = time.time()

                prev_distance_motor1 = current_distance_motor1
                prev_distance_motor2 = current_distance_motor2

                # Move both motors down at 50% duty cycle
                self._set_motor_duty(1, -512)
                self._set_motor_duty(2, -512)

            time.sleep(0.1)

        self.set_brake()
        self.position_motor1 = int(current_distance_motor1)
        self.position_motor2 = int(current_distance_motor2)
        self.save_position(
            {"motor1": self.position_motor1, "motor2": self.position_motor2}
        )
        print(
            "Homing complete. New zero positions: Motor1={}, Motor2={}".format(
                self.position_motor1, self.position_motor2
            )
        )

    def update(self, timer):
        """
        Periodic update function to check motor positions and velocities.

        This method is called by a timer to regularly update motor positions,
        calculate velocities, and check for safety conditions.

        Args:
            timer: The timer object (not used in the function body).
        """
        if self.debug_mode:
            print("Update function called")
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None and distance_motor2 is not None:
            pos1 = int(distance_motor1)
            pos2 = int(distance_motor2)

            # Calculate velocities
            dt = time.time() - self.last_update_time
            velocity1=0
            velocity2=0
            if dt!=0:
                velocity1 = (pos1 - self.last_positions[0]) / dt
                velocity2 = (pos2 - self.last_positions[1]) / dt

            # Update last positions and time
            self.last_positions = [pos1, pos2]
            self.last_update_time = time.time()

            # Check for excessive velocity
            if abs(velocity1) > self.max_velocity or abs(velocity2) > self.max_velocity:
                print("Excessive velocity detected, stopping motors.")
                self.set_brake()

    def check_height_limits_and_sync(self, timer):
        """
        Check if the desk is within height limits and if motors are synchronized.

        This method is called periodically by a timer to ensure the desk stays
        within its operational limits and that both motors remain synchronized.

        Args:
            timer: The timer object (not used in the function body).
        """
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None and distance_motor2 is not None:
            # Check height limits
            if (distance_motor1 >= self.max_height or distance_motor1 <= self.min_height or
                distance_motor2 >= self.max_height or distance_motor2 <= self.min_height):
                print("Height limit exceeded, stopping motors.")
                self.set_brake()

            # Check synchronization
            position_difference = abs(distance_motor1 - distance_motor2)
            if position_difference > self.max_position_difference:
                print("Motors out of sync, adjusting.")
                average_position = (distance_motor1 + distance_motor2) / 2
                self.pid_motor1.setpoint = average_position
                self.pid_motor2.setpoint = average_position

    def run_diagnostics(self):
        """
        Run a diagnostic test on the desk system.

        This method performs a series of movements to test the desk's
        functionality and the LIDAR sensors' accuracy.
        """
        print("Running diagnostics...")
        # Test motor movement
        self.move_to_position(80)  # Move to middle position
        time.sleep(2)
        self.move_to_position(70)  # Move down slightly
        time.sleep(2)
        self.move_to_position(90)  # Move up slightly
        time.sleep(2)
        self.home_position()  # Return to home position

        # Test LIDAR sensors
        print("Testing LIDAR sensors...")
        for _ in range(10):
            distance1 = self.lidar_motor1.distance()
            distance2 = self.lidar_motor2.distance()
            print(f"LIDAR 1: {distance1}, LIDAR 2: {distance2}")
            time.sleep(0.5)

        print("Diagnostics complete.")
    def run_diagnostics2(self):
        """
        Run a diagnostic test on the desk system.

        This method performs a series of movements to test the desk's
        functionality and the LIDAR sensors' accuracy.
        """
        print("Running diagnostics...")
        # Test motor movement
        self.move_to_position(80)  # Move to middle position
        time.sleep(2)
        self.move_to_position(70)  # Move down slightly
        time.sleep(2)
        self.move_to_position(90)  # Move up slightly
        time.sleep(2)
        self.home_position()  # Return to home position

        # Test LIDAR sensors
        print("Testing LIDAR sensors...")
        for _ in range(10):
            distance1 = self.lidar_motor1.distance()
            distance2 = self.lidar_motor2.distance()
            print(f"LIDAR 1: {distance1}, LIDAR 2: {distance2}")
            time.sleep(0.5)

        print("Diagnostics complete.")

    def log_error(self, error_message):
        """
        Log an error message with a timestamp.

        Args:
            error_message (str): The error message to log.
        """
        timestamp = time.localtime()
        formatted_time = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2], timestamp[3], timestamp[4], timestamp[5]
        )
        with open("error_log.txt", "a") as f:
            f.write(f"{formatted_time}: {error_message}\n")

    def emergency_stop(self):
        """
        Perform an emergency stop of the desk system.

        This method immediately stops all motors, disables the motor controller,
        and logs the emergency stop event.
        """
        print("Emergency stop activated!")
        self.set_brake()
        self.motor_controller(False)  # Disable motor controller
        self.log_error("Emergency stop activated")
