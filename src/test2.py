#!/usr/bin/env python3
from can_master.msg import can_in
import rospy
from payload_can.payload_lib import Payload
import time

# def test_high_curr_dc(id):
#     payload.DC_HIGH_CURR_write(id,0,65000)
#     time.sleep(3)
#     payload.DC_HIGH_CURR_write(id,1,65000)
#     time.sleep(3)
#     payload.DC_HIGH_CURR_write(id,0,0)

def test_dc(pub, id, id2):
    msg = can_in()
    msg.arb_id = id
    if id2 == 0:
        msg.data = b'\x00\xFF\xFF'
    if id2 == 1:
        msg.data = b'\x02\xFF\xFF'
    if id2 == 2:
        msg.data = b'\x04\xFF\xFF'
    pub.publish(msg)

# def test_pump(id):
#     payload.PUMP_write(id, 180)
#     time.sleep(3)
#     payload.PUMP_write(id, 0)
#     time.sleep(3)
#     payload.PUMP_write(id,90)

def test_GPIO_read(pub, id):
    msg = can_in()
    msg.arb_id = id
    msg.data = b'\x42\x00\x00'
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("test")
    payload = Payload()

    while not rospy.is_shutdown():
        payload.solenoid_write(2, True)
        time.sleep(0.1)
        payload.solenoid_write(2, False)
        time.sleep(1)
        # test_GPIO_read(can_pub, id)
    #test_high_curr_dc(id)
    #test_pump(id)
    #test_GPIO_read(id)
