import copy

import rospy
from std_msgs.msg import Int32, Int8, Float32MultiArray, UInt8MultiArray
import enum
import threading
import time
import sys


SENSOR_INDEX = {
            'center-left': 1,
            'center-right': 0,
            'rear-left': 3,
            'rear-right': 2,
            'side-left': 4,
            'side-right': 5
        }

DC_MOTOR = {
        'forward': Int8(data=3),
        'reverse': Int8(data=4),
        'left': Int8(data=1),
        'right': Int8(data=2),
        'stop': Int8(data=0),
        'cw': Int8(data=5),
        'ccw': Int8(data=6)
        }

lastMovement = DC_MOTOR['stop']

ir_mutex = threading.Lock()
Readings = []


def _sensor_callback(ir: UInt8MultiArray):
    global ir_mutex
    global Readings
    ir_mutex.acquire()
    Readings = ir.data
    ir_mutex.release()

def cleanRight(readings, motorPub):
    global lastMovement
    if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
        newMovement = DC_MOTOR['ccw']
        motorPub.publish(DC_MOTOR['ccw'])
        retVal = 1
    elif not readings[SENSOR_INDEX['side-right']]:
        newMovement = DC_MOTOR['stop']
        motorPub.publish(DC_MOTOR['stop'])
        retVal = -1
    else:
        newMovement = DC_MOTOR['right']
        motorPub.publish(DC_MOTOR['right'])
        retVal = 0
    if newMovement != lastMovement:
        motorPub.publish(newMovement)
        lastMovement = newMovement
    return retVal

def cleanLeft(readings, motorPub):
    global lastMovement
    if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
        newMovement = DC_MOTOR['cw']
        retVal = 1
    elif not readings[SENSOR_INDEX['side-left']]:
        newMovement = DC_MOTOR['stop']
        retVal = -1
    else:
        newMovement = DC_MOTOR['left']
        retVal = 0
    if newMovement != lastMovement:
        motorPub.publish(newMovement)
        lastMovement = newMovement
    return retVal

def main():
    global Readings
    global ir_mutex
    try:
        frequency = sys.argv[1]
    except:
        print('USAGE [frequency]')
        return
    rospy.init_node('horizontal_test')
    rospy.Subscriber('ir_sensor', UInt8MultiArray, _sensor_callback)
    time.sleep(1)
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
        rospy.Rate(1).sleep()


if __name__ == '__main__':
    main()
