import struct
import utime

class LIDAR:
    def __init__(self, uart, baud_rate=115200, timeout=100):
        self.uart = uart
        self.uart.init(baudrate=baud_rate, bits=8, parity=None, stop=1, timeout=timeout, rxbuf=256, txbuf=256)
        self.max_retries = 3
        
    def _send_command(self, command):
        self.uart.write(command)
        
    def _read_response(self, length, timeout_ms=100):
        start_time = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_time) < timeout_ms:
            if self.uart.any() >= length:
                return self.uart.read(length)
            utime.sleep_ms(10)
        return None
    
    def _calculate_checksum(self, data):
        return sum(data) & 0xFF
    
    def _verify_checksum(self, data):
        return self._calculate_checksum(data[:-1]) == data[-1]
    
    def _send_and_read(self, command, response_length, retries=3):
        for _ in range(retries):
            self._send_command(command)
            response = self._read_response(response_length)
            if response and len(response) == response_length:
                return response
            utime.sleep_ms(50)  # Wait before retry
        return None

    def _process_measurement(self, response):
        if response and len(response) == 9 and response[0] == 0x59 and response[1] == 0x59:
            if self._verify_checksum(response):
                dist = struct.unpack('<H', response[2:4])[0]
                amp = struct.unpack('<H', response[4:6])[0]
                temp = struct.unpack('<H', response[6:8])[0] / 8 - 256
                return {'Distance': dist, 'SignalAmp': amp, 'ChipTemp': temp}
        return None

    def measure(self):
        response = self._send_and_read(b'\x5A\x04\x04\x00', 9)
        return self._process_measurement(response)

    def distance(self):
        measurement = self.measure()
        return measurement['Distance'] if measurement else None

    def signal_amp(self):
        measurement = self.measure()
        return measurement['SignalAmp'] if measurement else None

    def temp(self):
        measurement = self.measure()
        return measurement['ChipTemp'] if measurement else None

    def version(self):
        response = self._send_and_read(b'\x5A\x04\x01\x00', 7)
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
        return self.measure()

    def print_payload_table(self):
        measurement = self.measure()
        if measurement:
            print("| Byte | 0   | 1   | 2       | 3       | 4       | 5       | 6       | 7       | 8         |")
            print("|------|-----|-----|---------|---------|---------|---------|---------|---------|-----------|")
            print("| Desc | 0x59| 0x59| Dist_L  | Dist_H  | Amp_L   | Amp_H   | Temp_L  | Temp_H  | Check_sum |")
            print(f"| Data | 59  | 59  | {measurement['Distance'] & 0xFF:02X}      | {measurement['Distance'] >> 8:02X}      | {measurement['SignalAmp'] & 0xFF:02X}      | {measurement['SignalAmp'] >> 8:02X}      | {int((measurement['ChipTemp'] + 256) * 8) & 0xFF:02X}      | {int((measurement['ChipTemp'] + 256) * 8) >> 8:02X}      | XX        |")
            print(f"\nDist: {measurement['Distance']} cm\nAmp: {measurement['SignalAmp']}\nTemp: {measurement['ChipTemp']:.2f}℃")
        else:
            print("Invalid response or no response received.")

    def set_amp_threshold(self, amp_threshold, dummy_dist):
        amp_threshold_byte = amp_threshold // 10
        command = struct.pack('<BBBBHB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, 0x00)
        checksum = self._calculate_checksum(command)
        command = struct.pack('<BBBBHBB', 0x5A, 0x07, 0x22, amp_threshold_byte, dummy_dist, checksum)
        self._send_command(command)
        self._send_command(b'\x5A\x04\x11\x00')  # Save settings
