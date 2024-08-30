import struct
import utime
import time
import constants as const
from machine import UART

class LIDAR:
    def __init__(self, uart, baud_rate=115200):
        self.uart = uart
        self.uart.init(baudrate=baud_rate, bits=8, parity=None, stop=1)
        self.dist=0
        self.temp=0
        self.amp=0
        
    def _send_command(self, command):
        self.uart.write(command)
        
    def _read_response(self, length):
        return self.uart.read(length)
    
    def _calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def _verify_checksum(self, data):
        return self._calculate_checksum(data[:-1]) == data[-1]
    
    def distance(self):
        self.dist,self.temp,self.amp=self.read_all()
        return self.amp

    def signal_amp(self):
        self.dist,self.temp,self.amp=self.read_all()
        return self.amp

    def temp(self):
        self.dist,self.temp,self.amp=self.read_all()
        return self.temp

    def set_frequency(self, freq=100):
        command = struct.pack('<BBHB', 0x5A, 0x06, 0x03, freq, 0x00)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings

    def power_saving_mode(self, power_saving_mode=True):
        val = 0x01 if power_saving_mode else 0x00
        command = struct.pack('<BBBBB', 0x5A, 0x05, 0x35, val, 0x00)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings

    def on_off(self, on=True):
        val = 0x01 if on else 0x00
        command = struct.pack('<BBBBB', 0x5A, 0x05, 0x07, val, 0x00)
        self._send_command(command)

    def reset(self):
        self._send_command(b'\x5A\x04\x10\x00')  # Restore factory settings
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings

    def set_min_max(self, min_dist, max_dist):
        command = struct.pack('<BBHHB', 0x5A, 0x09, 0x3A, min_dist, max_dist, 0x00)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings

    def read_all(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        retries=10
        for i in range(0,retries):
            if response:
                if self._verify_checksum(response) and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
                        self.dist = struct.unpack('<H', response[2:4])[0]
                        self.amp = struct.unpack('<H', response[4:6])[0]
                        self.temp = struct.unpack('<H', response[6:8])[0]
                        return {"Distance":self.dist,"Amplitude":self.amp,"Temperature":self.temp}
                else:
                    print("Invalid response or bad checksum "+str(response))
            else:
                print("No response")
            print("Retrying "+str(i))            
            time.sleep(0.5)
