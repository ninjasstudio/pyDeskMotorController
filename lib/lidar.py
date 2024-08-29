import struct
import utime
import time
import constants as const
from machine import UART

class LIDAR:
    def __init__(self, uart, baud_rate=115200):
        self.uart = uart
        self.uart.init(baudrate=baud_rate, bits=8, parity=None, stop=1)
        
    def _send_command(self, command):
        self.uart.write(command)
        
    def _read_response(self, length):
        return self.uart.read(length)
    
    def _calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def _verify_checksum(self, data):
        return self._calculate_checksum(data[:-1]) == data[-1]
    
    def distance(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            if self._verify_checksum(response):
                return dist
        return None

    def signal_amp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            amp = struct.unpack('<H', response[4:6])[0]
            if self._verify_checksum(response):
                return amp
        return None

    def temp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return temp / 8 - 256
        return None

    def version(self):
        self._send_command(b'\x5A\x04\x01\x00')  # Get version
        response = self._read_response(7)
        if response and len(response) == 7 and response[0] == 0x5A and response[2] == 0x01:
            return 'LiDAR Version {}.{}.{}'.format(response[5], response[4], response[3])
        return None

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
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            amp = struct.unpack('<H', response[4:6])[0]
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return 'Distance {}, ChipTemp {}, SignalAmp {}'.format(
                    dist, temp / 8 - 256, amp)
        return None

    def print_payload_table(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            amp = struct.unpack('<H', response[4:6])[0]
            temp = struct.unpack('<H', response[6:8])[0]
            checksum = response[8]
            if self._verify_checksum(response):
                print("| Byte | 0   | 1   | 2       | 3       | 4       | 5       | 6       | 7       | 8         |")
                print("|------|-----|-----|---------|---------|---------|---------|---------|---------|-----------|")
                print("| Desc | 0x59| 0x59| Dist_L  | Dist_H  | Amp_L   | Amp_H   | Temp_L  | Temp_H  | Check_sum |")
                print(f"| Data | {response[0]:02X}  | {response[1]:02X}  | {response[2]:02X}      | {response[3]:02X}      | {response[4]:02X}      | {response[5]:02X}      | {response[6]:02X}      | {response[7]:02X}      | {response[8]:02X}       |")
                print(f"\nDist: {dist} cm\nAmp: {amp}\nTemp: {temp / 8 - 256:.2f}℃\nChecksum: {checksum}")
            else:
                print("Checksum verification failed.")
        else:
            print("Invalid response or no response received.")

    def set_amp_threshold(self, amp_threshold, dummy_dist):
        amp_threshold_byte = amp_threshold // 10
        command = struct.pack('<BBBBHB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, 0x00)
        checksum = self._calculate_checksum(command)
        command = struct.pack('<BBBBHBB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, checksum)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings
import struct
import utime
import time
import constants as const
from machine import UART

class LIDAR:
    def __init__(self, uart, baud_rate=115200):
        self.uart = uart
        self.uart.init(baudrate=baud_rate, bits=8, parity=None, stop=1)
        
    def _send_command(self, command):
        self.uart.write(command)
        
    def _read_response(self, length):
        return self.uart.read(length)
    
    def _calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def _verify_checksum(self, data):
        return self._calculate_checksum(data[:-1]) == data[-1]
    
    def distance(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            if self._verify_checksum(response):
                return dist
        return None

    def signal_amp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            amp = struct.unpack('<H', response[4:6])[0]
            if self._verify_checksum(response):
                return amp
        return None

    def temp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return temp / 8 - 256
        return None

    def version(self):
        self._send_command(b'\x5A\x04\x01\x00')  # Get version
        response = self._read_response(7)
        if response and len(response) == 7 and response[0] == 0x5A and response[2] == 0x01:
            return 'LiDAR Version {}.{}.{}'.format(response[5], response[4], response[3])
        return None

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
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            amp = struct.unpack('<H', response[4:6])[0]
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return {'Distance':dist, 'ChipTemp':temp, 'SignalAmp':amp}

            return {"response":str(response)}
        return {"response"}
        
        

    def print_payload_table(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            amp = struct.unpack('<H', response[4:6])[0]
            temp = struct.unpack('<H', response[6:8])[0]
            checksum = response[8]
            if self._verify_checksum(response):
                print("| Byte | 0   | 1   | 2       | 3       | 4       | 5       | 6       | 7       | 8         |")
                print("|------|-----|-----|---------|---------|---------|---------|---------|---------|-----------|")
                print("| Desc | 0x59| 0x59| Dist_L  | Dist_H  | Amp_L   | Amp_H   | Temp_L  | Temp_H  | Check_sum |")
                print(f"| Data | {response[0]:02X}  | {response[1]:02X}  | {response[2]:02X}      | {response[3]:02X}      | {response[4]:02X}      | {response[5]:02X}      | {response[6]:02X}      | {response[7]:02X}      | {response[8]:02X}       |")
                print(f"\nDist: {dist} cm\nAmp: {amp}\nTemp: {temp / 8 - 256:.2f}℃\nChecksum: {checksum}")
            else:
                print("Checksum verification failed.")
        else:
            print("Invalid response or no response received.")

    def set_amp_threshold(self, amp_threshold, dummy_dist):
        amp_threshold_byte = amp_threshold // 10
        command = struct.pack('<BBBBHB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, 0x00)
        checksum = self._calculate_checksum(command)
        command = struct.pack('<BBBBHBB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, checksum)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings
