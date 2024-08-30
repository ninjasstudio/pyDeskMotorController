from lib.desk_controller import DeskController
import time

def main() -> None:
    print("Initializing Desk")
    desk = DeskController(debug=True)
    print("Setting Limits")
        
    desk.lidar_motor1.set_min_max(40,150)
    desk.lidar_motor1.set_frequency(100)
    desk.lidar_motor1.set_amp_threshold(3000,40)
    desk.lidar_motor2.set_min_max(40,150)
    desk.lidar_motor2.set_frequency(100)
    desk.lidar_motor2.set_amp_threshold(3000,40)    
    time.sleep(1)
    

    while True: 
        for i in range(0,65535):
            dist1=0
            dist2=0
            
            data2 = desk.lidar_motor2.read_all()            
            time.sleep(1)
            if data2 is not None and len(data2)>1:
                print("Lidar Motor 2 : "+str(data2))
                dist2=data2["Distance"]
            elif data2 is not None:                
                print("Lidar Motor 2 : "+str(data2))
            else:
                print("Lidar Motor 2 : "+"None")

            data1 = desk.lidar_motor1.read_all()            
            time.sleep(1)
            if data1 is not None and len(data1)>1:
                print("Lidar Motor 1 : "+str(data1))
                dist1=data1["Distance"]
            elif data1 is not None:                
                print("Lidar Motor 1 : "+str(data1))
            else:
                print("Lidar Motor 1 : "+"None")
  

        #last_position = desk.load_position()
        desk.position_motor1 = dist1
        desk.position_motor2 = dist2
        desk.motor_controller(True)
        time.sleep(1)
   #$     print("Motor 1 50%") 
# 8
#         print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
#         print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
#         time.sleep(0.1)       
#         desk._set_motor_duty(1, 512)
#         time.sleep(1)
#         desk.set_brake()
#         print("Motor 1 -50%")        
#         dist1 = desk.lidar_motor1.distance()
#         dist2 = desk.lidar_motor2.distance()
#         print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
#         print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
#         desk._set_motor_duty(1, -512)
#         time.sleep(1)
#         desk.set_brake()
#         print("Motor 2 50%")        
#         dist1 = desk.lidar_motor1.distance()
#         dist2 = desk.lidar_motor2.distance()
#         print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
#         print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")
#         desk._set_motor_duty(2, 512)
#         time.sleep(1)
#         desk.set_brake()


#         print("Motor 2 -50%")        
#         desk._set_motor_duty(2, -512)
#         time.sleep(1)
#         desk.set_brake()
#         dist1 = desk.lidar_motor1.distance()
#         dist2 = desk.lidar_motor2.distance()
#         print(f"LIDAR 1 distance: {dist1 if dist1 is not None else 'No reading'}")
#         print(f"LIDAR 2 distance: {dist2 if dist2 is not None else 'No reading'}")


        desk.run_diagnostics()
        desk.move_to_position(57) 
        desk.emergency_stop()

if __name__ == "__main__":
    main()
