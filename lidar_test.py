from machine import UART
from lib.lidar import LIDAR
uart1 = UART(1, baudrate=38400, tx=27, rx=14)
uart2 = UART(2, baudrate=38400, tx=25, rx=26)
lidar_motor1 = LIDAR(uart1)
lidar_motor2 = LIDAR(uart2)
lidar_motor1.distance
lidar_motor2.distance