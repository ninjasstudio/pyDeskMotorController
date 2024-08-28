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

    # Quick motor direction test
    print("Testing motor directions...")

    # Move motors up slightly with easing
    print("Moving motors up...")
    desk.move_motor_smooth(1, 512, 2)  # Move motor 1 up smoothly over 2 seconds
    desk.move_motor_smooth(2, 512, 2)  # Move motor 2 up smoothly over 2 seconds

    # Read distances after moving up
    dist1_up = desk.lidar_motor1.distance()
    dist2_up = desk.lidar_motor2.distance()
    print(f"LIDAR 1 distance after moving up: {dist1_up if dist1_up is not None else 'No reading'}")
    print(f"LIDAR 2 distance after moving up: {dist2_up if dist2_up is not None else 'No reading'}")

    # Move motors down slightly with easing
    print("Moving motors down...")
    desk.move_motor_smooth(1, -512, 2)  # Move motor 1 down smoothly over 2 seconds
    desk.move_motor_smooth(2, -512, 2)  # Move motor 2 down smoothly over 2 seconds

    # Read distances after moving down
    dist1_down = desk.lidar_motor1.distance()
    dist2_down = desk.lidar_motor2.distance()
    print(f"LIDAR 1 distance after moving down: {dist1_down if dist1_down is not None else 'No reading'}")
    print(f"LIDAR 2 distance after moving down: {dist2_down if dist2_down is not None else 'No reading'}")

    print("Motor direction test complete.")

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
