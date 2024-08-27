from lib.desk_controller import DeskController
import time

def main() -> None:
    # 25.2 inch min
    # 50.8 inch max
    # 25.6 inch range
    # = 65.024 cm
    desk = DeskController()
    desk.lidar_motor1.set_min_max(64, 129)
    desk.lidar_motor1.set_frequency(120)
    desk.lidar_motor2.set_min_max(64, 129)
    desk.lidar_motor2.set_frequency(120)
        # Min and max height
        #self.min_height = 64.008  # Define appropriate min height
        #self.max_height = 129.032  # Define appropriate max height
    while True:
        print(desk.lidar_motor1.distance())
        print(desk.lidar_motor2.distance())
        time.sleep(1)
        last_position = desk.load_position()
        desk.position_motor1 = last_position["motor1"]
        desk.position_motor2 = last_position["motor2"]
        desk.motor_controller(True)
        desk.set_break()
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
