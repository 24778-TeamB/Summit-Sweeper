import rospy
import std_msgs
import threading
from ticlib import TicUSB, TIC36v4

mtx1 = threading.Lock()
mtx2 = threading.Lock()

frontTic = TicUSB(product=TIC36v4, serial_number='00414637')
# rearTic = TicUSB(product=TIC36v4, serial_number=None)


def callbackSetPos1(pos: std_msgs.msg.Int32):
    mtx1.acquire()
    frontTic.set_target_position(pos.data)
    mtx1.release()


def callbackSetPos2(pos: std_msgs.msg.Int32):
    mtx2.acquire()
    # rearTic.set_target_position(pos.data)
    mtx2.release()


def main():
    pass
