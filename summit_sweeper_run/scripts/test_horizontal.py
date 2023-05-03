import copy

import rospy
from std_msgs.msg import UInt8MultiArray
import enum
import threading
import time
import sys
import requests
from typing import Tuple, Dict


class horizontalSpeeds(dict):
    def __init__(self, url: str):
        super().__init__()
        frontRight, frontLeft, midRight, midLeft, rearRight, rearLeft = self._load_speed_configs(url)
        _preparedMessages = {
            'forward': UInt8MultiArray(data=[3, frontRight['forward'], frontLeft['forward'], midRight['forward'],
                                             midLeft['forward'], rearRight['forward'], rearLeft['forward']]),
            'reverse': UInt8MultiArray(data=[4, frontRight['reverse'], frontLeft['reverse'], midRight['reverse'],
                                             midLeft['reverse'], rearRight['reverse'], rearLeft['reverse']]),
            'left': UInt8MultiArray(data=[2, frontRight['left'], frontLeft['left'], midRight['left'],
                                          midLeft['left'], rearRight['left'], rearLeft['left']]),
            'right': UInt8MultiArray(data=[1, frontRight['right'], frontLeft['right'], midRight['right'],
                                           midLeft['right'], rearRight['right'], rearLeft['right']]),
            'cw': UInt8MultiArray(data=[3, frontRight['cw'], frontLeft['cw'], midRight['cw'],
                                        midLeft['cw'], rearRight['cw'], rearLeft['cw']]),
            'ccw': UInt8MultiArray(data=[3, frontRight['ccw'], frontLeft['ccw'], midRight['ccw'],
                                         midLeft['ccw'], rearRight['ccw'], rearLeft['ccw']]),
            'climb': UInt8MultiArray(data=[3, frontRight['climb'], frontLeft['climb'], midRight['climb'],
                                           midLeft['climb'], rearRight['climb'], rearLeft['climb']]),
            'stop': UInt8MultiArray(data=[0, frontRight['stop'], frontLeft['stop'], midRight['stop'],
                                          midLeft['stop'], rearRight['stop'], rearLeft['stop']]),
        }
        self.__dict__ = _preparedMessages

    @staticmethod
    def _load_speed_configs(url) -> Tuple[Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int]]:
        frontRight = {}
        frontLeft = {}
        midRight = {}
        midLeft = {}
        rearRight = {}
        rearLeft = {}
        f = requests.get(url)
        data = f.json()
        data = data['motor-speeds']
        for motor in data:
            if motor['motor-set'] == 'front' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    frontRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'front' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    frontLeft[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'middle' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    midRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'middle' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    midLeft[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'rear' and motor['side'] == 'right':
                for speeds in motor['speeds']:
                    rearRight[speeds['movement']] = speeds['speed']
            if motor['motor-set'] == 'rear' and motor['side'] == 'left':
                for speeds in motor['speeds']:
                    rearLeft[speeds['movement']] = speeds['speed']
        return frontRight, frontLeft, midRight, midLeft, rearRight, rearLeft

    def __getitem__(self, key):
        return self.__dict__[key]

    def __repr__(self):
        return repr(self.__dict__)


SENSOR_INDEX = {
            'center-left': 1,
            'center-right': 0,
            'rear-left': 3,
            'rear-right': 2,
            'side-left': 4,
            'side-right': 5
        }

DC_MOTOR = horizontalSpeeds('https://raw.githubusercontent.com/24778-TeamB/motor-speeds/master/speeds.json')

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
            self._startDirection = self.Direction.RIGHT
        else:
            self._startDirection = self.Direction.LEFT
        self._cleanDirection = self._startDirection

    def _correctOrientation(self, readings):
        # Both sensors are off the wall, move forward
        if readings[SENSOR_INDEX['center-left']] and readings[SENSOR_INDEX['center-right']]:
            self.direction = self.Direction.FORWARD
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.FORWARD
                rospy.loginfo('Forward')
                self.dc_motors.publish(DC_MOTOR['forward'])
        # Right sensor off the wall, rotate CCW
        elif readings[SENSOR_INDEX['center-right']] and not readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CW
                rospy.loginfo('cw')
                self.dc_motors.publish(DC_MOTOR['cw'])
        # Left sensor off the wall, rotate CW
        elif not readings[SENSOR_INDEX['center-right']] and readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CCW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CCW
                self.dc_motors.publish(DC_MOTOR['ccw'])
                rospy.loginfo('ccw')
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
                rospy.loginfo('stop')
            self._completedStates['right'] = True
            return True
        self.direction = self.Direction.RIGHT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.RIGHT
            self.dc_motors.publish(DC_MOTOR['right'])
            rospy.loginfo('right')
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
                rospy.loginfo('stop')
            self._completedStates['left'] = True
            return True
        self.direction = self.Direction.LEFT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.LEFT
            self.dc_motors.publish(DC_MOTOR['left'])
            rospy.loginfo('left')
        return False

    def resetStateMachine(self) -> None:
        self.dc_motors.publish(DC_MOTOR['stop'])
        self.direction = self.Direction.STOP
        self.lastMovement = self.Direction.STOP
        self._completedStates['left'] = False
        self._completedStates['right'] = False
        self._newStep = True
        rospy.loginfo('Reset state machine')
        return

    def next(self, readings) -> bool:
        if self._newStep:
            self._newStep = False
            self.direction = self._startDirection
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


def wait_for_subscribers(horizontal):
    i = 0
    while not rospy.is_shutdown() and horizontal.get_num_connections() == 0:
        if i == 4:
            if horizontal.get_num_connections() == 0:
                rospy.loginfo(f'Waiting for subscriber to connect to {horizontal.name}')
        rospy.Rate(10).sleep()
        i += 1
        i %= 5
    if rospy.is_shutdown():
        raise Exception('Got shutdown request before subscribers could connect')
    return


def main():
    global Readings
    global ir_mutex
    try:
        frequency = float(sys.argv[1])
    except:
        print('USAGE [frequency]')
        return
    rospy.init_node('horizontal_test', disable_signals=True)
    rospy.Subscriber('ir_sensor', UInt8MultiArray, _sensor_callback)
    time.sleep(1)
    horizontal_movement = rospy.Publisher('horizontal_control', UInt8MultiArray, queue_size=4)
    wait_for_subscribers(horizontal_movement)
    stateMachine = HorizontalMovement(horizontal_movement, True)

    while not rospy.is_shutdown():
        try:
            while not stateMachine.next(Readings) and not rospy.is_shutdown():
                rospy.loginfo(stateMachine.direction)
                rospy.Rate(frequency).sleep()
            rospy.loginfo('Completed step')
            stateMachine.resetStateMachine()
        except KeyboardInterrupt:
            horizontal_movement.publish(DC_MOTOR['stop'])
            rospy.signal_shutdown('Shutting down node')


if __name__ == '__main__':
    main()
