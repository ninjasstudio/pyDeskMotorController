from machine import Timer
import time
from desk_controller import DeskController

def main() -> None:
    desk = DeskController()    
    desk.lidar_motor1.set_min_max(20, 150)
    desk.lidar_motor1.set_frequency(250)
    desk.lidar_motor2.set_min_max(20, 150)
    desk.lidar_motor2.set_frequency(250)

    while True:
        print(desk.lidar_motor1.distance())
        print(desk.lidar_motor2.distance())
        time.sleep(1)
    #last_position = desk.load_position()
    #desk.position_motor1 = last_position["motor1"]
    #desk.position_motor2 = last_position["motor2"]
    #desk.motor_controller(True)
    #desk.set_break()
    #desk.home_position()   
    #presets = [100, 200, 300]
    #for preset in presets:
    #    desk.move_to_position(preset)
    #    time.sleep(2)  # Wait for 2 seconds at each preset
    # Example usage
    
    desk = DeskController()
    desk.set_debug_mode(True)
    desk.run_up(1, 512, 2)  # Run motor 1 up with 50% duty cycle for 2 seconds
    desk.run_down(2, 512, 2)  # Run motor 2 down with 50% duty cycle for 2 seconds
    
    desk.save_position({"motor1": desk.position_motor1, "motor2": desk.position_motor2})    
    desk.motor_controller(enable_power=False)


if __name__ == '__main__':
    main()
