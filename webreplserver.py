
import network
import time
from machine import Pin


def connect_to_wifi():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        wlan.active(True)
        wlan.connect("Don't Panic!", "4or+yTw0!")

        while not wlan.isconnected():
            print('Connecting to network...')
            time.sleep(1)
    if wlan.isconnected():
        print('Network connected!')
        print('IP Address:', wlan.ifconfig()[0])
    else:
        print("Wifi not connected")

import webrepl_setup

def main():
    connect_to_wifi()
    import webrepl

    webrepl.start()

    

if __name__ == '__main__':
    main()
