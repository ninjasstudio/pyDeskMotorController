from lib.desk_controller import DeskController
import time

def main() -> None:
    print("Initializing Desk")
    desk = DeskController()
    print("Setting Limits")
    desk.lidar_motor1.set_min_max(64, 129)
    desk.lidar_motor1.set_frequency(100)
    desk.lidar_motor2.set_min_max(64, 129)
    desk.lidar_motor2.set_frequency(100)

    print("Starting main loop")
    while True:
        dist1 = desk.lidar_motor1.distance()
        dist2 = desk.lidar_motor2.distance()
        print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
        print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
        time.sleep(1)
        
        last_position = desk.load_position()
        desk.position_motor1 = last_position["motor1"]
        desk.position_motor2 = last_position["motor2"]
        desk.motor_controller(True)
        desk.run_diagnostics()
        desk.move_to_position(57) 
        desk.emergency_stop()

if __name__ == "__main__":
    main()
