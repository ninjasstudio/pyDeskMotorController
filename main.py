from lib.desk_controller import DeskController
import time
# +----------------+-------------+
# | Component      | GPIO Pin    |
# +----------------+-------------+
# | Left Motor     |             |
# |   PWM Up       | GPIO 5      |
# |   PWM Dn       | GPIO 4      |
# |   LOW EN       | GPIO 2      |
# +----------------+-------------+
# | Right Motor    |             |
# |   PWM Up       | GPIO 23     |
# |   PWM Dn       | GPIO 22     |
# |   LOW EN       | GPIO 21     |
# +----------------+-------------+
# | Lidar Left     |             |
# |   SCL/TX       | GPIO 27     |
# |   SDA/RX       | GPIO 14     |
# +----------------+-------------+
# | Lidar Right    |             |
# |   SCL/TX       | GPIO 26     |
# |   SDA/RX       | GPIO 25     |
# +----------------+-------------+

# +-------------------------------+-------------------+
# | Specification                 | Value             |
# +-------------------------------+-------------------+
# | Desk Min                      | 25.2 in / 64.0 cm |
# | Desk Max                      | 50.8 in / 129.0 cm|
# | Desk Range                    | 25.6 in / 65.0 cm |
# +-------------------------------+-------------------+
# | Lidar Range                   | 6cm <-> 300 cm±6cm|
# | Amp                           | 2000              |
# | Laser Diameter at 129cm       | 3cm               |
# | Lidar max rate                | 400kbps i2c       |
# +-------------------------------+-------------------+
# | PWM Freq                      | 10khz             |
# | Motor Voltage                 | 33v               |
# | Power Supply Voltage          | 24v               |
# +-------------------------------+-------------------+
# | Proportional Gain             | 1.1               |
# | Integral Gain                 | 0.1               |
# | Derivative Gain               | 0.1               |
# +-------------------------------+-------------------+
# | Lidar Max diff before         |                   |
# | correction                    | 3cm               |
# +-------------------------------+-------------------+
# | Lidar Max diff before panic   | 6cm               |
# +-------------------------------+-------------------+
# | Max run time                  | 15 sec            |
# | Cool off time                 | 90 sec            |
# +-------------------------------+-------------------+

def main() -> None:


    print("Initializing Desk")
    desk = DeskController()
    print("Setting Limits")
    desk.lidar_motor1.set_min_max(64, 129)
    desk.lidar_motor1.set_frequency(100)
    desk.lidar_motor2.set_min_max(64, 129)
    desk.lidar_motor2.set_frequency(100)
    desk.min_height = 64
    desk.max_height = 129

    print("Starting main loop")
    while True:
        print(desk.lidar_motor1.distance())
        print(desk.lidar_motor2.distance())
        time.sleep(1)
        last_position = desk.load_position()
        desk.position_motor1 = last_position["motor1"]
        desk.position_motor2 = last_position["motor2"]
        desk.motor_controller(True)
        desk.set_break()
        print("Homing...")
        desk.home_position()
        presets = [80, 100, 120]
        for preset in presets:
            desk.move_to_position(preset)
            time.sleep(20)  # Wait for 2 seconds at each preset
            desk.save_position({"motor1": desk.position_motor1, "motor2": desk.position_motor2})
    # Example usage

    #desk.set_debug_mode(True)
    #desk.run_up(1, 512, 2)  # Run motor 1 up with 50% duty cycle for 2 seconds
    #desk.run_down(2, 512, 2)  # Run motor 2 down with 50% duty cycle for 2 seconds

    desk.motor_controller(enable_power=False)


if __name__ == "__main__":
    main()
