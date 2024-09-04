from lib.desk_controller import DeskController
import time

def main() -> None:
    print("Initializing Desk")
    desk = DeskController()

    print("Starting main loop")
    while True:
        # Read distances from both sensors
        distance1 = desk.sonar_motor1.distance_mm()
        distance2 = desk.sonar_motor2.distance_mm()

        # Display distances
        print(f"Sonar 1 distance: {distance1} mm")
        print(f"Sonar 2 distance: {distance2} mm")
        print("-" * 30)  # Separator for readability

        # Wait for 1 second before next reading
        time.sleep(1)

if __name__ == "__main__":
    main()
