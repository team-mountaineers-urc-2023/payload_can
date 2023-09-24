#!/usr/bin/env python3
from payload_lib import Payload
import os
import time

def test_high_curr_dc(id):
    payload.DC_HIGH_CURR_write(id,0,65000)
    time.sleep(3)
    payload.DC_HIGH_CURR_write(id,1,65000)
    time.sleep(3)
    payload.DC_HIGH_CURR_write(id,0,0)

def test_dc(id):
    payload.DC_write(id,0,60000)
    time.sleep(3)
    payload.DC_write(id,1,60000)
    time.sleep(3)
    payload.DC_write(id,0,0)

def test_pump(id):
    payload.PUMP_write(id, 180)
    time.sleep(3)
    payload.PUMP_write(id, 0)
    time.sleep(3)
    payload.PUMP_write(id,90)

def test_GPIO_read(id):
    message = payload.GPIO_read(id)
    print(message)


if __name__ == "__main__":
    print('Bring up CAN0....\n')
    try:
        os.system("sudo /sbin/ip link set can0 down")
        time.sleep(0.1)
    except Exception as e:
        print(e)

    try:
        os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
        time.sleep(0.1)
    except Exception as e:
        print(e)

    payload = Payload()

    id = 1

    #test_dc(id)
    #test_high_curr_dc(id)
    #test_pump(id)
    #test_GPIO_read(id)
