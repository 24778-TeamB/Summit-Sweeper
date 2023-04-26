import copy

import rospy
from std_msgs.msg import Int32, Int8, Float32MultiArray, UInt8MultiArray
import enum
import threading
import time


SENSOR_INDEX = {
            'center-left': 1,
            'center-right': 0,
            'rear-left': 3,
            'rear-right': 2,
            'side-left': 5,
            'side-right': 4
        }

DC_MOTOR = {
        'forward': Int8(data=3),
        'reverse': Int8(data=4),
        'left': Int8(data=1),
        'right': Int8(data=2),
        'stop': Int8(data=0)
        }

ir_mutex = threading.Lock()
Readings = []


def _sensor_callback(ir: UInt8MultiArray):
    global ir_mutex
    global Readings
    ir_mutex.acquire()
    Readings = ir.data
    ir_mutex.release()

def cleanRight(readings, motorPub):
    if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
        motorPub.publish(DC_MOTOR['forward'])
        return 1
    elif not readings[SENSOR_INDEX['side-right']]:
        motorPub.publish(DC_MOTOR['stop'])
        return -1
    else:
        motorPub.publish(DC_MOTOR['right'])
    return 0

def cleanLeft(readings, motorPub):
    if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
        motorPub.publish(DC_MOTOR['forward'])
        return 1
    elif not readings[SENSOR_INDEX['side-left']]:
        motorPub.publish(DC_MOTOR['stop'])
        return -1
    else:
        motorPub.publish(DC_MOTOR['left'])
    return 0

def main():
    global Readings
    global ir_mutex
    rospy.Subscriber('ir_sensor', UInt8MultiArray, _sensor_callback)
    horizontal_movement = rospy.Publisher('horizontal_control', Int8, queue_size=4)
    cleanRight(Readings, horizontal_movement)
    state = True

    while not rospy.is_shutdown():
        ir_mutex.acquire()
        readings = copy.deepcopy(Readings)
        ir_mutex.release()
        if state:
            result = cleanRight(readings, horizontal_movement)
            state = result != -1
        else:
            result = cleanLeft(readings, horizontal_movement)
            state = result == -1
        rospy.Rate(10).sleep()


if __name__ == '__main__':
    main()
