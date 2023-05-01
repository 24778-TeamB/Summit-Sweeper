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


class HorizontalMovement:
    class Direction(enum.Enum):
        STOP = 0x0
        FORWARD = 0x1
        LEFT = 0x2
        RIGHT = 0x3
        CW = 0x4
        CCW = 0x5

    def __init__(self, dcMotorPub, startingPosLeft: bool):
        self.direction = self.Direction.STOP
        self._completedStates = {'left': False, 'right': False}
        self.dc_motors = dcMotorPub
        self.lastMovement = self.Direction.STOP
        self._newStep = True
        if startingPosLeft:
            self._startPos = self.Direction.LEFT
        else:
            self._startPos = self.Direction.RIGHT
        self._cleanDirection = self._startPos

    def _correctOrientation(self, readings):
        # Both sensors are off the wall, move forward
        if readings[SENSOR_INDEX['center-left']] and readings[SENSOR_INDEX['center-right']]:
            self.direction = self.Direction.FORWARD
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.FORWARD
                self.dc_motors.publish(DC_MOTOR['forward'])
        # Right sensor off the wall, rotate CCW
        elif readings[SENSOR_INDEX['center-right']] and not readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CCW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CCW
                self.dc_motors.publish(DC_MOTOR['ccw'])
        # Left sensor off the wall, rotate CW
        elif not readings[SENSOR_INDEX['center-right']] and readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CW
                self.dc_motors.publish(DC_MOTOR['cw'])
        return

    def _cleanRight(self, readings) -> bool:
        if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
            self._correctOrientation(readings)
            return False
        elif not readings[SENSOR_INDEX['side-right']]:
            self.direction = self.Direction.STOP
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.STOP
                self.dc_motors.publish(DC_MOTOR['stop'])
            self._completedStates['right'] = True
            return True
        self.direction = self.Direction.RIGHT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.RIGHT
            self.dc_motors.publish(DC_MOTOR['right'])
        return False

    def _cleanLeft(self, readings) -> bool:
        if readings[SENSOR_INDEX['center-right']] or readings[SENSOR_INDEX['center-left']]:
            self._correctOrientation(readings)
            return False
        elif not readings[SENSOR_INDEX['side-left']]:
            self.direction = self.Direction.STOP
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.STOP
                self.dc_motors.publish(DC_MOTOR['stop'])
            self._completedStates['left'] = True
            return True
        self.direction = self.Direction.LEFT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.LEFT
            self.dc_motors.publish(DC_MOTOR['left'])
        return False

    def resetStateMachine(self) -> None:
        self.dc_motors.publish(DC_MOTOR['stop'])
        self.direction = self.Direction.STOP
        self.lastMovement = self.Direction.STOP
        self._completedStates['left'] = False
        self._completedStates['right'] = False
        self._newStep = True
        return

    def next(self, readings) -> bool:
        if self._newStep:
            self._newStep = False
            if self._startPos == self.Direction.LEFT:
                self._cleanDirection = self.Direction.LEFT
            else:
                self._cleanDirection = self.Direction.RIGHT
        if self._cleanDirection == self.Direction.LEFT:
            if self._cleanLeft(readings):
                self._cleanDirection = self.Direction.RIGHT
        elif self._cleanDirection == self.Direction.RIGHT:
            if self._cleanRight(readings):
                self._cleanDirection = self.Direction.LEFT
        else:
            rospy.logerr(f'Invalid horizontal movement state: {self._cleanDirection}')
        return self._completedStates['right'] and self._completedStates['left']


def _sensor_callback(ir: UInt8MultiArray):
    global ir_mutex
    global Readings
    ir_mutex.acquire()
    Readings = ir.data
    ir_mutex.release()


def main():
    global Readings
    global ir_mutex
    try:
        frequency = float(sys.argv[1])
    except:
        print('USAGE [frequency]')
        return
    rospy.init_node('horizontal_test')
    rospy.Subscriber('ir_sensor', UInt8MultiArray, _sensor_callback)
    time.sleep(1)
    horizontal_movement = rospy.Publisher('horizontal_control', Int8, queue_size=4)
    stateMachine = HorizontalMovement(horizontal_movement, True)

    while not rospy.is_shutdown():
        while stateMachine.next(Readings):
            rospy.Rate(frequency).sleep()
        stateMachine.resetStateMachine()


if __name__ == '__main__':
    main()
