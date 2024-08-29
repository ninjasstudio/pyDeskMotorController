from lib.desk_controller import DeskController
import time

def main() -> None:
    print("Initializing Desk")
    desk = DeskController()
    print("Setting Limits")
    desk.lidar_motor1.set_min_max(50, 129)
    desk.lidar_motor1.set_frequency(50)
    desk.lidar_motor2.set_min_max(50, 129)
    desk.lidar_motor2.set_frequency(50)
    desk.lidar_motor1.signal_amp=300
    desk.lidar_motor2.signal_amp=300
    desk.lidar_motor1.read_all()
    print(str(desk.lidar_motor1.print_payload_table()))
    print(str(desk.lidar_motor2.print_payload_table()))

    # Quick motor direction test
    #$print("Testing motor directions...")

    # Move motors up slightly with easing
    #print("Moving motors up...")
    #desk.move_motor_smooth(1, 512, 2)  # Move motor 1 up smoothly over 2 seconds
    #desk.move_motor_smooth(2, 512, 2)  # Move motor 2 up smoothly over 2 seconds

    # Read distances after moving up
    dist1_up = desk.lidar_motor1.distance()
    dist2_up = desk.lidar_motor2.distance()
    print(f"LIDAR 1 distance after moving up: {dist1_up if dist1_up is not None else 'No reading'}")
    print(f"LIDAR 2 distance after moving up: {dist2_up if dist2_up is not None else 'No reading'}")

    # Move motors down slightly with easing
    #print("Moving motors down...")
    #desk.move_motor_smooth(1, -512, 2)  # Move motor 1 down smoothly over 2 seconds
    #desk.move_motor_smooth(2, -512, 2)  # Move motor 2 down smoothly over 2 seconds

    # Read distances after moving down
    #dist1_down = desk.lidar_motor1.distance()
    #dist2_down = desk.lidar_motor2.distance()
    #print(f"LIDAR 1 distance after moving down: {dist1_down if dist1_down is not None else 'No reading'}")
    #print(f"LIDAR 2 distance after moving down: {dist2_down if dist2_down is not None else 'No reading'}")

    #print("Motor direction test complete.")

    #print("Starting main loop")
    desk.lidar_motor1.set_min_max(40,150)
    desk.lidar_motor1.set_frequency(100)
    desk.lidar_motor1.set_amp_threshold(300,40)
    desk.lidar_motor2.set_min_max(40,150)
    desk.lidar_motor2.set_frequency(100)
    desk.lidar_motor2.set_amp_threshold(300,40)
    while True: 
        for i in range(0,65535):
            data1 = desk.lidar_motor1.read_all()            
            time.sleep(1)
            if data1 is not None and len(data1)>1:
                dist1=data1["Distance"]
                temp1=data1["ChipTemp"]
                amp1=data1["SignalAmp"]
            else:
                response=data["response"]
            
            

            dist2 = desk.lidar_motor2.distance()
            time.sleep(1)
            
            time.sleep(1)  
            print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
            time.sleep(1)
            print(str(desk.lidar_motor2.print_payload_table()))
            time.sleep(1)
            print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
            time.sleep(1)   
        
        #last_position = desk.load_position()
        desk.position_motor1 = dist1
        desk.position_motor2 = dist2
        desk.motor_controller(True)
        time.sleep(1)
        print("Motor 1 50%") 
        dist1 = desk.lidar_motor1.distance()
        dist2 = desk.lidar_motor2.distance()
        print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
        print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
        time.sleep(0.1)       
        desk._set_motor_duty(1, 512)
        time.sleep(1)
        desk.set_brake()
        print("Motor 1 -50%")        
        dist1 = desk.lidar_motor1.distance()
        dist2 = desk.lidar_motor2.distance()
        print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
        print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
        desk._set_motor_duty(1, -512)
        time.sleep(1)
        desk.set_brake()
        print("Motor 2 50%")        
        dist1 = desk.lidar_motor1.distance()
        dist2 = desk.lidar_motor2.distance()
        print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
        print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
        desk._set_motor_duty(2, 512)
        time.sleep(1)
        desk.set_brake()


        print("Motor 2 -50%")        
        desk._set_motor_duty(2, -512)
        time.sleep(1)
        desk.set_brake()
        dist1 = desk.lidar_motor1.distance()
        dist2 = desk.lidar_motor2.distance()
        print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
        print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")


        desk.run_diagnostics()
        desk.move_to_position(57) 
        desk.emergency_stop()

if __name__ == "__main__":
    main()
