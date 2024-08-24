from machine import Pin, PWM, Timer
import time
import json
from simple_pid import PID
from typing import Dict

class DeskController:
    def __init__(self):
        # Motor pins
        self.motor1_enable: Pin = Pin(22, Pin.OUT, Pin.PULL_UP, value=1)
        self.motor2_enable: Pin = Pin(23, Pin.OUT, Pin.PULL_UP, value=1)
        
        # Hall effect sensor pins
        self.hall_sensor_motor1_a: Pin = Pin(34, Pin.IN)
        self.hall_sensor_motor1_b: Pin = Pin(35, Pin.IN)
        self.hall_sensor_motor2_a: Pin = Pin(32, Pin.IN)
        self.hall_sensor_motor2_b: Pin = Pin(33, Pin.IN)
        
        # Define PWM frequency
        pwm_freq: int = 10000  # 10kHz
        
        # Initialize PWM channels
        self.pwm_motor1_in1: PWM = PWM(Pin(15), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor1_in2: PWM = PWM(Pin(18), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor2_in1: PWM = PWM(Pin(19), freq=pwm_freq, duty=0, channel=0)
        self.pwm_motor2_in2: PWM = PWM(Pin(21), freq=pwm_freq, duty=0, channel=0)
        
        # Variables to store the last pulse times
        self.last_pulse_time_motor1_a: int = 0
        self.last_pulse_time_motor1_b: int = 0
        self.last_pulse_time_motor2_a: int = 0
        self.last_pulse_time_motor2_b: int = 0
        self.debounce_interval: int = 100000  # Debounce interval in nanoseconds
        
        # Position tracking
        self.position_motor1: int = 0
        self.position_motor2: int = 0
        
        # PID controller
        self.pid: PID = PID(1.0, 0.1, 0.01, setpoint=0)
        self.pid.output_limits = (0, 1023)  # Limit output to PWM range
        
        # Non-volatile memory (NVM) storage
        self.nvm_file: str = "desk_position.json"
        
        # Debug mode
        self.debug_mode: bool = False
        
        # Set up hall sensor interrupts
        self.hall_sensor_motor1_a.irq(trigger=Pin.IRQ_RISING, handler=self.hall_sensor_callback)
        self.hall_sensor_motor1_b.irq(trigger=Pin.IRQ_RISING, handler=self.hall_sensor_callback)
        self.hall_sensor_motor2_a.irq(trigger=Pin.IRQ_RISING, handler=self.hall_sensor_callback)
        self.hall_sensor_motor2_b.irq(trigger=Pin.IRQ_RISING, handler=self.hall_sensor_callback)

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

    @micropython.viper
    def hall_sensor_callback(self, pin: int) -> None:
        if self.debug_mode:
            return
        
        current_time: int = int(time.time_ns())
        
        if pin == int(self.hall_sensor_motor1_a):
            if current_time - self.last_pulse_time_motor1_a > self.debounce_interval:
                self.last_pulse_time_motor1_a = current_time
                self.position_motor1 += 1
        elif pin == int(self.hall_sensor_motor1_b):
            if current_time - self.last_pulse_time_motor1_b > self.debounce_interval:
                self.last_pulse_time_motor1_b = current_time
                self.position_motor1 -= 1
        elif pin == int(self.hall_sensor_motor2_a):
            if current_time - self.last_pulse_time_motor2_a > self.debounce_interval:
                self.last_pulse_time_motor2_a = current_time
                self.position_motor2 += 1
        elif pin == int(self.hall_sensor_motor2_b):
            if current_time - self.last_pulse_time_motor2_b > self.debounce_interval:
                self.last_pulse_time_motor2_b = current_time
                self.position_motor2 -= 1

    @micropython.native
    def set_break(self) -> None:
        print("Setting break: stopping all motors")
        self.pwm_motor1_in1.duty(0)
        self.pwm_motor1_in2.duty(0)
        self.pwm_motor2_in1.duty(0)
        self.pwm_motor2_in2.duty(0)

    @micropython.native
    def run_down(self, duty_cycle: int, duration: float, motor_id: int = None) -> None:
        if motor_id is None:
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
    def run_up(self, duty_cycle: int, duration: float, motor_id: int = None) -> None:
        if motor_id is None:
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
    def _ramp_pwm_up(self, target_duty_cycle: int, motor_id: int = None) -> None:
        step_size = 10
        for duty_cycle in range(0, target_duty_cycle + 1, step_size):
            if motor_id is None:
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
    def _ramp_pwm_down(self, motor_id: int = None) -> None:
        step_size = 10
        for duty_cycle in range(100, -1, -step_size):
            if motor_id is None:
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
        self.pid.setpoint = target_position
        while abs(self.position_motor1 - target_position) > 1 or abs(self.position_motor2 - target_position) > 1:
            if self.position_motor1 is not None:
                pos1: int = int(self.position_motor1)
                duty_cycle: int = int(self.pid(pos1))
                if self.position_motor1 < target_position:
                    self.run_up(duty_cycle, 0.1, 1)
                else:
                    self.run_down(duty_cycle, 0.1, 1)
            if self.position_motor2 is not None:
                pos2: int = int(self.position_motor2)
                duty_cycle: int = int(self.pid(pos2))
                if self.position_motor2 < target_position:
                    self.run_up(duty_cycle, 0.1, 2)
                else:
                    self.run_down(duty_cycle, 0.1, 2)

    @micropython.native
    def home_position(self) -> None:
        print("Homing...")
        # Implement homing logic here
        # For simplicity, assume homing sets both positions to zero
        self.position_motor1 = 0
        self.position_motor2 = 0
        self.save_position({"motor1": self.position_motor1, "motor2": self.position_motor2})

    # Safety feature: Check for pulse reception
    @micropython.native
    def check_pulses(self, timer: Timer) -> None:
        current_time: int = time.time_ns()
        if (current_time - self.last_pulse_time_motor1_a > 500_000_000 and
            current_time - self.last_pulse_time_motor1_b > 500_000_000 and
            current_time - self.last_pulse_time_motor2_a > 500_000_000 and
            current_time - self.last_pulse_time_motor2_b > 500_000_000):
            print("No pulses detected for 0.5 seconds, stopping motors.")
            self.set_break()

    # Update function to be called at a fixed interval
    @micropython.native
    def update(self, timer: Timer) -> None:
        # Add any periodic tasks you want to perform here
        print("Update function called")

# Example usage
desk = DeskController()
desk.set_debug_mode(True)
desk.run_up(512, 2)  # Run both motors up with 50% duty cycle for 2 seconds
desk.run_down(512, 2, 1)  # Run motor 1 down with 50% duty cycle for 2 seconds
