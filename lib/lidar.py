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
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            if self._verify_checksum(response):
                return dist
        return None

    def signal_amp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            amp = struct.unpack('<H', response[4:6])[0]
            if self._verify_checksum(response):
                return amp
        return None

    def temp(self):
        self._send_command(b'\x5A\x04\x04\x00')  # Trigger measurement
        response = self._read_response(9)
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return temp / 8 - 256
        return None

    def version(self):
        self._send_command(b'\x5A\x04\x01\x00')  # Get version
        response = self._read_response(7)
        if len(response) == 7 and response[0] == 0x5A and response[2] == 0x01:
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
        if len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            dist = struct.unpack('<H', response[2:4])[0]
            amp = struct.unpack('<H', response[4:6])[0]
            temp = struct.unpack('<H', response[6:8])[0]
            if self._verify_checksum(response):
                return 'Distance {}, ChipTemp {}, SignalAmp {}'.format(
                    dist, temp / 8 - 256, amp)
        return None
