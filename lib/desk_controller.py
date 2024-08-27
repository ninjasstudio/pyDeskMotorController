
from machine import Pin, PWM, Timer, I2C
import time
import json
from lib.PID import PID
from lib import lidar as Lidar

class DeskController:
    def __init__(self):
        # Motor pins
        self.motor1_enable = Pin(4, Pin.OUT, Pin.PULL_UP, value=1)
        self.motor2_enable = Pin(5, Pin.OUT, Pin.PULL_UP, value=1)

        # Define PWM frequency
        pwm_freq = 10000  # 10kHz

        # Initialize PWM channels
        self.pwm_motor1_in1 = PWM(Pin(15), freq=pwm_freq, duty=0)
        self.pwm_motor1_in2 = PWM(Pin(18), freq=pwm_freq, duty=0)
        self.pwm_motor2_in1 = PWM(Pin(2), freq=pwm_freq, duty=0)
        self.pwm_motor2_in2 = PWM(Pin(23), freq=pwm_freq, duty=0)

        # Position tracking
        self.position_motor1 = 0
        self.position_motor2 = 0

        # PID controllers
        self.pid_motor1 = PID(1.0, 0.1, 0.01, setpoint=0)
        self.pid_motor2 = PID(1.0, 0.1, 0.01, setpoint=0)
        self.pid_motor1.output_limits = (0, 1023)  # Limit output to PWM range
        self.pid_motor2.output_limits = (0, 1023)  # Limit output to PWM range

        # Non-volatile memory (NVM) storage
        self.nvm_file = "desk_position.json"

        # Debug mode
        self.debug_mode = False

        # LIDAR addresses
        self.LIDAR_ADDRESS_1 = 0x10
        self.LIDAR_ADDRESS_2 = 0x10

        # Initialize I2C for TF Luna sensors
        self.i2c = I2C(0, scl=Pin(27), sda=Pin(14), freq=400000)
        self.i2c2 = I2C(1, scl=Pin(25), sda=Pin(26), freq=400000)

        

        print(str(self.i2c) + ": " + str(self.i2c2))
        time.sleep(1)
        slaves = self.i2c.scan()
        slaves2 = self.i2c2.scan()
        print("Slaves = "+str(slaves) + ":" + str(slaves2)) 
        if self.LIDAR_ADDRESS_1 not in slaves or self.LIDAR_ADDRESS_2 not in slaves2:
            print("Bus error: Please check LIDAR wiring")

        self.lidar_motor1 = Lidar.LIDAR(self.i2c, slaves[0])
        self.lidar_motor2 = Lidar.LIDAR(self.i2c2, slaves[0])


        # Min and max height
        self.min_height = 0  # Define appropriate min height
        self.max_height = 1000  # Define appropriate max height

        # Timer for checking height limits
        self.limit_check_timer = Timer(-1)
        self.limit_check_timer.init(
            period=100, mode=Timer.PERIODIC, callback=self.check_height_limits
        )

        # Timer for periodic updates
        self.update_timer = Timer(-1)
        self.update_timer.init(period=100, mode=Timer.PERIODIC, callback=self.update)

        # Timer for printing PID components
        self.print_pid_timer = Timer(-1)
        self.print_pid_timer.init(
            period=5000, mode=Timer.PERIODIC, callback=self.print_pid_components
        )

    def set_debug_mode(self, debug):
        self.debug_mode = debug
        print("Debug mode set to {}".format(self.debug_mode))

    def save_position(self, position):
        with open(self.nvm_file, "w") as f:
            json.dump(position, f)

    def load_position(self):
        try:
            with open(self.nvm_file, "r") as f:
                return json.load(f)
        except OSError:
            return {"motor1": 0, "motor2": 0}

    def set_break(self):
        print("Setting break: stopping all motors")
        self.pwm_motor1_in1.duty(0)
        self.pwm_motor1_in2.duty(0)
        self.pwm_motor2_in1.duty(0)
        self.pwm_motor2_in2.duty(0)

    def run_down(self, duty_cycle, duration, motor_id=0):
        print(
            "Running down {} with duty cycle {} for {} seconds".format(
                'both motors' if motor_id == 0 else 'motor {}'.format(motor_id),
                duty_cycle,
                duration
            )
        )
        self.ramp_pwm_up(duty_cycle, motor_id)
        self._set_motor_duty(motor_id, duty_cycle, direction="down")
        time.sleep(duration)
        self.set_break()
        self._ramp_pwm_down(motor_id)

    def run_up(self, duty_cycle, duration, motor_id=0):
        print(
            "Running up {} with duty cycle {} for {} seconds".format(
                'both motors' if motor_id == 0 else 'motor {}'.format(motor_id),
                duty_cycle,
                duration
            )
        )
        self.ramp_pwm_up(duty_cycle, motor_id)
        self._set_motor_duty(motor_id, duty_cycle, direction="up")
        time.sleep(duration)
        self.set_break()
        self._ramp_pwm_down(motor_id)

    def _set_motor_duty(self, motor_id, duty_cycle, direction):
        if motor_id == 0:
            self.pwm_motor1_in1.duty(0 if direction == "up" else duty_cycle)
            self.pwm_motor1_in2.duty(duty_cycle if direction == "up" else 0)
            self.pwm_motor2_in1.duty(0 if direction == "up" else duty_cycle)
            self.pwm_motor2_in2.duty(duty_cycle if direction == "up" else 0)
        elif motor_id == 1:
            self.pwm_motor1_in1.duty(0 if direction == "up" else duty_cycle)
            self.pwm_motor1_in2.duty(duty_cycle if direction == "up" else 0)
        elif motor_id == 2:
            self.pwm_motor2_in1.duty(0 if direction == "up" else duty_cycle)
            self.pwm_motor2_in2.duty(duty_cycle if direction == "up" else 0)

    def ramp_pwm_up(self, target_duty_cycle, motor_id=0):
        step_size = 10
        for duty_cycle in range(0, target_duty_cycle + 1, step_size):
            self._set_motor_duty(motor_id, duty_cycle, direction="up")
            print(
                "Ramping up PWM for {}: {}%".format(
                    'both motors' if motor_id == 0 else 'motor {}'.format(motor_id),
                    duty_cycle
                )
            )
            time.sleep(0.1)

    def _ramp_pwm_down(self, motor_id=0):
        step_size = 10
        for duty_cycle in range(100, -1, -step_size):
            self._set_motor_duty(motor_id, duty_cycle, direction="down")
            print(
                "Ramping down PWM for {}: {}%".format(
                    'both motors' if motor_id == 0 else 'motor {}'.format(motor_id),
                    duty_cycle
                )
            )
            time.sleep(0.1)

    def motor_controller(self, enable_power):
        print(
            "Setting motor controller to {}".format('enabled' if enable_power else 'disabled')
        )
        enable_power_active_low = not enable_power
        self.motor1_enable.value(enable_power_active_low)
        self.motor2_enable.value(enable_power_active_low)

    def move_to_position(self, target_position):
        self.pid_motor1.setpoint = target_position
        self.pid_motor2.setpoint = target_position
        while True:
            distance_motor1 = self.lidar_motor1.distance()
            distance_motor2 = self.lidar_motor2.distance()

            if distance_motor1 is not None:
                pos1 = int(distance_motor1)
                duty_cycle = int(self.pid_motor1(pos1))
                if distance_motor1 < target_position:
                    self.run_up(duty_cycle, 0.1, 1)
                else:
                    self.run_down(duty_cycle, 0.1, 1)

            if distance_motor2 is not None:
                pos2 = int(distance_motor2)
                duty_cycle = int(self.pid_motor2(pos2))
                if distance_motor2 < target_position:
                    self.run_up(duty_cycle, 0.1, 2)
                else:
                    self.run_down(duty_cycle, 0.1, 2)

            # Break condition to avoid infinite loop
            if (
                distance_motor1 is not None
                and abs(distance_motor1 - target_position) <= 1
                and distance_motor2 is not None
                and abs(distance_motor2 - target_position) <= 1
            ):
                break

    def home_position(self):
        print("Homing...")

        prev_distance_motor1 = self.lidar_motor1.distance()
        prev_distance_motor2 = self.lidar_motor2.distance()
        stable_time_start = time.time()

        while True:
            current_distance_motor1 = self.lidar_motor1.distance()
            current_distance_motor2 = self.lidar_motor2.distance()

            if (
                current_distance_motor1 is not None
                and current_distance_motor2 is not None
            ):
                if (
                    current_distance_motor1 >= prev_distance_motor1
                    and current_distance_motor2 >= prev_distance_motor2
                ):
                    stable_time_elapsed = time.time() - stable_time_start
                    if stable_time_elapsed >= 1.0:
                        break
                else:
                    stable_time_start = time.time()

                prev_distance_motor1 = current_distance_motor1
                prev_distance_motor2 = current_distance_motor2

                self.run_down(512, 0.1)  # Run both motors down with 50% duty cycle

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
        print("Update function called")
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None and distance_motor2 is not None:
            pos1 = int(distance_motor1)
            pos2 = int(distance_motor2)

            # Calculate the difference between the two positions
            position_difference = abs(pos1 - pos2)

            # If the difference exceeds a threshold, adjust the PID setpoints
            if position_difference > 5:  # Threshold value, can be adjusted
                average_position = (pos1 + pos2) // 2
                self.pid_motor1.setpoint = average_position
                self.pid_motor2.setpoint = average_position
                print("Synchronizing motors: new setpoint {}".format(average_position))

    def check_height_limits(self, timer):
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if (
            distance_motor1 is not None
            and (
                distance_motor1 >= self.max_height or distance_motor1 <= self.min_height
            )
        ) or (
            distance_motor2 is not None
            and (
                distance_motor2 >= self.max_height or distance_motor2 <= self.min_height
            )
        ):
            print("Height limit exceeded, stopping motors.")
            self.set_break()

    def print_pid_components(self, timer):
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None:
            pos1 = int(distance_motor1)
            p1 = self.pid_motor1.Kp * (self.pid_motor1.setpoint - pos1)
            i1 = self.pid_motor1._integral
            d1 = self.pid_motor1._derivative
            print("Motor 1 PID components: P={}, I={}, D={}".format(p1, i1, d1))
            print(distance_motor1)
            print(str(self.lidar_motor1))

        if distance_motor2 is not None:
            pos2 = int(distance_motor2)
            p2 = self.pid_motor2.Kp * (self.pid_motor2.setpoint - pos2)
            i2 = self.pid_motor2._integral
            d2 = self.pid_motor2._derivative
            print("Motor 2 PID components: P={}, I={}, D={}".format(p2, i2, d2))
            print(distance_motor1)
            print(str(self.lidar_motor1))