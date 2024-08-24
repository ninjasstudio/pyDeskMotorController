from machine import Pin, PWM, Timer, I2C
import time
import json
from lib.PID import PID
from lib import lidar as Lidar
from typing import Dict

class DeskController:
    def __init__(self):
        # Motor pins
        self.motor1_enable: Pin = Pin(22, Pin.OUT, Pin.PULL_UP, value=1)
        self.motor2_enable: Pin = Pin(23, Pin.OUT, Pin.PULL_UP, value=1)
        
        # Define PWM frequency
        pwm_freq: int = 10000  # 10kHz
        
        # Initialize PWM channels
        self.pwm_motor1_in1: PWM = PWM(Pin(15), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor1_in2: PWM = PWM(Pin(18), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor2_in1: PWM = PWM(Pin(19), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor2_in2: PWM = PWM(Pin(21), freq=pwm_freq, duty=0, channel=0)
        
        # Position tracking
        self.position_motor1: int = 0
        self.position_motor2: int = 0
        
        # PID controllers
        self.pid_motor1: PID = PID(1.0, 0.1, 0.01, setpoint=0)
        self.pid_motor2: PID = PID(1.0, 0.1, 0.01, setpoint=0)
        self.pid_motor1.output_limits = (0, 1023)  # Limit output to PWM range
        self.pid_motor2.output_limits = (0, 1023)  # Limit output to PWM range
        
        # Non-volatile memory (NVM) storage
        self.nvm_file: str = "desk_position.json"
        
        # Debug mode
        self.debug_mode: bool = False
        
        # LIDAR addresses
        self.LIDAR_ADDRESS_1 = 0x10
        self.LIDAR_ADDRESS_2 = 0x20
        
        # Initialize I2C for TF Luna sensors
        self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
        time.sleep(1)
        slaves = self.i2c.scan()
        if self.LIDAR_ADDRESS_1 or self.LIDAR_ADDRESS_2 not in slaves:
            print('Bus error: Please check LIDAR wiring')
    
        self.lidar_motor1 = Lidar.LIDAR(self.i2c, self.LIDAR_ADDRESS_1)
        self.lidar_motor2 = Lidar.LIDAR(self.i2c, self.LIDAR_ADDRESS_2)        
        print(self.lidar_motor1.version())
        print(self.lidar_motor2.version())
        
        # Min and max height
        self.min_height: int = 0  # Define appropriate min height
        self.max_height: int = 1000  # Define appropriate max height

        # Timer for checking height limits
        self.limit_check_timer = Timer(-1)
        self.limit_check_timer.init(period=100, mode=Timer.PERIODIC, callback=self.check_height_limits)

        # Timer for periodic updates
        self.update_timer = Timer(-1)
        self.update_timer.init(period=100, mode=Timer.PERIODIC, callback=self.update)

        # Timer for printing PID components
        self.print_pid_timer = Timer(-1)
        self.print_pid_timer.init(period=5000, mode=Timer.PERIODIC, callback=self.print_pid_components)

    def set_debug_mode(self, debug: bool) -> None:
        self.debug_mode = debug
        print(f"Debug mode set to {self.debug_mode}")

    @micropython.native
    def save_position(self, position: Dict[str, int]) -> None:
        with open(self.nvm_file, 'w') as f:
            json.dump(position, f)

    @micropython.native
    def load_position(self) -> Dict[str, int]:
        try:
            with open(self.nvm_file, 'r') as f:
                return json.load(f)
        except OSError:
            return {"motor1": 0, "motor2": 0}

    @micropython.native
    def set_break(self) -> None:
        print("Setting break: stopping all motors")
        self.pwm_motor1_in1.duty(0)
        self.pwm_motor1_in2.duty(0)
        self.pwm_motor2_in1.duty(0)
        self.pwm_motor2_in2.duty(0)

    @micropython.native
    def run_down(self, duty_cycle: int, duration: float, motor_id: int = 0) -> None:
        if motor_id == 0:
            print(f"Running down both motors with duty cycle {duty_cycle} for {duration} seconds")
            self._ramp_pwm_up(duty_cycle)
            self.pwm_motor1_in1.duty(duty_cycle)
            self.pwm_motor1_in2.duty(0)
            self.pwm_motor2_in1.duty(duty_cycle)
            self.pwm_motor2_in2.duty(0)
        else:
            print(f"Running down motor {motor_id} with duty cycle {duty_cycle} for {duration} seconds")
            self._ramp_pwm_up(duty_cycle, motor_id)
            if motor_id == 1:
                self.pwm_motor1_in1.duty(duty_cycle)
                self.pwm_motor1_in2.duty(0)
            elif motor_id == 2:
                self.pwm_motor2_in1.duty(duty_cycle)
                self.pwm_motor2_in2.duty(0)
        time.sleep(duration)
        self.set_break()
        self._ramp_pwm_down(motor_id)

    @micropython.native
    def run_up(self, duty_cycle: int, duration: float, motor_id: int = 0) -> None:
        if motor_id == 0:
            print(f"Running up both motors with duty cycle {duty_cycle} for {duration} seconds")
            self._ramp_pwm_up(duty_cycle)
            self.pwm_motor1_in1.duty(0)
            self.pwm_motor1_in2.duty(duty_cycle)
            self.pwm_motor2_in1.duty(0)
            self.pwm_motor2_in2.duty(duty_cycle)
        else:
            print(f"Running up motor {motor_id} with duty cycle {duty_cycle} for {duration} seconds")
            self._ramp_pwm_up(duty_cycle, motor_id)
            if motor_id == 1:
                self.pwm_motor1_in1.duty(0)
                self.pwm_motor1_in2.duty(duty_cycle)
            elif motor_id == 2:
                self.pwm_motor2_in1.duty(0)
                self.pwm_motor2_in2.duty(duty_cycle)
        time.sleep(duration)
        self.set_break()
        self._ramp_pwm_down(motor_id)

    @micropython.native
    def _ramp_pwm_up(self, target_duty_cycle: int, motor_id: int = 0) -> None:
        step_size = 10
        for duty_cycle in range(0, target_duty_cycle + 1, step_size):
            if motor_id == 0:
                self.pwm_motor1_in1.duty(duty_cycle)
                self.pwm_motor1_in2.duty(duty_cycle)
                self.pwm_motor2_in1.duty(duty_cycle)
                self.pwm_motor2_in2.duty(duty_cycle)
                print(f"Ramping up PWM for both motors: {duty_cycle}%")
            else:
                if motor_id == 1:
                    self.pwm_motor1_in1.duty(duty_cycle)
                    self.pwm_motor1_in2.duty(duty_cycle)
                elif motor_id == 2:
                    self.pwm_motor2_in1.duty(duty_cycle)
                    self.pwm_motor2_in2.duty(duty_cycle)
                print(f"Ramping up PWM for motor {motor_id}: {duty_cycle}%")
            time.sleep(0.1)

    @micropython.native
    def _ramp_pwm_down(self, motor_id: int = 0) -> None:
        step_size = 10
        for duty_cycle in range(100, -1, -step_size):
            if motor_id == 0:
                self.pwm_motor1_in1.duty(duty_cycle)
                self.pwm_motor1_in2.duty(duty_cycle)
                self.pwm_motor2_in1.duty(duty_cycle)
                self.pwm_motor2_in2.duty(duty_cycle)
                print(f"Ramping down PWM for both motors: {duty_cycle}%")
            else:
                if motor_id == 1:
                    self.pwm_motor1_in1.duty(duty_cycle)
                    self.pwm_motor1_in2.duty(duty_cycle)
                elif motor_id == 2:
                    self.pwm_motor2_in1.duty(duty_cycle)
                    self.pwm_motor2_in2.duty(duty_cycle)
                print(f"Ramping down PWM for motor {motor_id}: {duty_cycle}%")
            time.sleep(0.1)

    @micropython.native
    def motor_controller(self, enable_power: bool) -> None:
        print(f"Setting motor controller to {'enabled' if enable_power else 'disabled'}")
        enable_power_active_low: bool = not enable_power
        self.motor1_enable.value(enable_power_active_low)
        self.motor2_enable.value(enable_power_active_low)

    @micropython.native
    def move_to_position(self, target_position: int) -> None:
        self.pid_motor1.setpoint = target_position
        self.pid_motor2.setpoint = target_position
        while True:
            distance_motor1 = self.lidar_motor1.distance()
            distance_motor2 = self.lidar_motor2.distance()

            if distance_motor1 is not None:
                pos1: int = int(distance_motor1)
                duty_cycle: int = int(self.pid_motor1(pos1))
                if distance_motor1 < target_position:
                    self.run_up(duty_cycle, 0.1, 1)
                else:
                    self.run_down(duty_cycle, 0.1, 1)

            if distance_motor2 is not None:
                pos2: int = int(distance_motor2)
                duty_cycle: int = int(self.pid_motor2(pos2))
                if distance_motor2 < target_position:
                    self.run_up(duty_cycle, 0.1, 2)
                else:
                    self.run_down(duty_cycle, 0.1, 2)

            # Break condition to avoid infinite loop
            if (distance_motor1 is not None and abs(distance_motor1 - target_position) <= 1 and 
                distance_motor2 is not None and abs(distance_motor2 - target_position) <= 1):
                break

    @micropython.native
    def home_position(self) -> None:
        print("Homing...")

        prev_distance_motor1 = self.lidar_motor1.distance()
        prev_distance_motor2 = self.lidar_motor2.distance()
        stable_time_start = time.time()

        while True:
            current_distance_motor1 = self.lidar_motor1.distance()
            current_distance_motor2 = self.lidar_motor2.distance()

            if current_distance_motor1 is not None and current_distance_motor2 is not None:
                if (current_distance_motor1 >= prev_distance_motor1 and current_distance_motor2 >= prev_distance_motor2):
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
        self.save_position({"motor1": self.position_motor1, "motor2": self.position_motor2})
        print(f"Homing complete. New zero positions: Motor1={self.position_motor1}, Motor2={self.position_motor2}")

    # Update function to be called at a fixed interval
    @micropython.native
    def update(self, timer: Timer) -> None:
        print("Update function called")
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None and distance_motor2 is not None:
            pos1: int = int(distance_motor1)
            pos2: int = int(distance_motor2)

            # Calculate the difference between the two positions
            position_difference = abs(pos1 - pos2)

            # If the difference exceeds a threshold, adjust the PID setpoints
            if position_difference > 5:  # Threshold value, can be adjusted
                average_position = (pos1 + pos2) // 2
                self.pid_motor1.setpoint = average_position
                self.pid_motor2.setpoint = average_position
                print(f"Synchronizing motors: new setpoint {average_position}")

    @micropython.native
    def check_height_limits(self, timer: Timer) -> None:
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if (distance_motor1 is not None and (distance_motor1 >= self.max_height or distance_motor1 <= self.min_height)) or \
           (distance_motor2 is not None and (distance_motor2 >= self.max_height or distance_motor2 <= self.min_height)):
            print("Height limit exceeded, stopping motors.")
            self.set_break()

    @micropython.native
    def print_pid_components(self, timer: Timer) -> None:
        distance_motor1 = self.lidar_motor1.distance()
        distance_motor2 = self.lidar_motor2.distance()

        if distance_motor1 is not None:
            pos1: int = int(distance_motor1)
            p1 = self.pid_motor1.Kp * (self.pid_motor1.setpoint - pos1)
            i1 = self.pid_motor1._integral
            d1 = self.pid_motor1._derivative
            print(f"Motor 1 PID components: P={p1}, I={i1}, D={d1}")

        if distance_motor2 is not None:
            pos2: int = int(distance_motor2)
            p2 = self.pid_motor2.Kp * (self.pid_motor2.setpoint - pos2)
            i2 = self.pid_motor2._integral
            d2 = self.pid_motor2._derivative
            print(f"Motor 2 PID components: P={p2}, I={i2}, D={d2}")

# Example usage
desk = DeskController()
desk.set_debug_mode(True)
desk.home_position()  # Perform homing operation
desk.run_up(512, 2)  # Run both motors up with 50% duty cycle for 2 seconds
desk.run_down(512, 2, 1)  # Run motor 1 down with 50% duty cycle for 2 seconds
