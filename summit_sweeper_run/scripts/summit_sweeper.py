import rospy
from std_msgs.msg import UInt8MultiArray, Int8, Int32
import threading
import copy
import enum
from typing import Dict, Tuple
import requests

VACUUM = {
    'on': Int8(data=1),
    'off': Int8(data=0),
    'test': Int8(data=2)
}

SENSOR_INDEX = {
    'center-left': 1,
    'center-right': 0,
    'rear-left': 3,
    'rear-right': 2,
    'side-left': 4,
    'side-right': 5
}


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
    def _load_speed_configs(url) -> Tuple[
        Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int], Dict[str, int]]:
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


class HorizontalMovement:
    class Direction(enum.Enum):
        STOP = 0x0
        FORWARD = 0x1
        LEFT = 0x2
        RIGHT = 0x3
        CW = 0x4
        CCW = 0x5

    def __init__(self, dcMotorPub, startingPosLeft: bool, speed_profile: horizontalSpeeds, roundTrip: bool):
        self.direction = self.Direction.STOP
        self._completedStates = {'left': False, 'right': False}
        self.dc_motors = dcMotorPub
        self.lastMovement = self.Direction.STOP
        self._newStep = True
        self._roundTrip = roundTrip
        if startingPosLeft:
            self._startDirection = self.Direction.RIGHT
        else:
            self._startDirection = self.Direction.LEFT
        self._cleanDirection = self._startDirection
        self.dc_movements = speed_profile

    def _correctOrientation(self, readings):
        # Both sensors are off the wall, move forward
        if readings[SENSOR_INDEX['center-left']] and readings[SENSOR_INDEX['center-right']]:
            self.direction = self.Direction.FORWARD
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.FORWARD
                rospy.loginfo('Forward')
                self.dc_motors.publish(self.dc_movements['forward'])
        # Right sensor off the wall, rotate CCW
        elif readings[SENSOR_INDEX['center-right']] and not readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CW
                rospy.loginfo('cw')
                self.dc_motors.publish(self.dc_movements['cw'])
        # Left sensor off the wall, rotate CW
        elif not readings[SENSOR_INDEX['center-right']] and readings[SENSOR_INDEX['center-left']]:
            self.direction = self.Direction.CCW
            if self.direction != self.lastMovement:
                self.lastMovement = self.Direction.CCW
                self.dc_motors.publish(self.dc_movements['ccw'])
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
                self.dc_motors.publish(self.dc_movements['stop'])
                rospy.loginfo('stop')
            self._completedStates['right'] = True
            return True
        self.direction = self.Direction.RIGHT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.RIGHT
            self.dc_motors.publish(self.dc_movements['right'])
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
                self.dc_motors.publish(self.dc_movements['stop'])
                rospy.loginfo('stop')
            self._completedStates['left'] = True
            return True
        self.direction = self.Direction.LEFT
        if self.direction != self.lastMovement:
            self.lastMovement = self.Direction.LEFT
            self.dc_motors.publish(self.dc_movements['left'])
            rospy.loginfo('left')
        return False

    def resetStateMachine(self) -> None:
        self.dc_motors.publish(self.dc_movements['stop'])
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
        return (self._completedStates['right'] and self._completedStates['left']) or \
            (self._completedStates['right'] and not self._roundTrip) or (self._completedStates['left'] and
                                                                         not self._roundTrip)


class stepStateMachine:
    class climbState(enum.Enum):
        CLEAN = 0X0
        LIFT_MIDDLE = 0X1
        FORWARD1 = 0X2
        LIFT_ENDS = 0x3
        FORWARD2 = 0X4

    def __init__(self, dc_motor_pub, speed_profile: horizontalSpeeds, vacuum, frontL=0, rearL=0, frontH=1, rearH=1,
                 stopStage1: bool = False, stopStage2: bool = False):
        self.frontTargets = {'low': Int32(data=frontL), 'high': Int32(data=frontH)}
        self.rearTargets = {'low': Int32(data=rearL), 'high': Int32(data=rearH)}
        self.mtx1 = threading.Lock()
        self.mtx2 = threading.Lock()
        self.frontPos = 0
        self.rearPos = 0
        self.currentState = self.climbState.CLEAN
        self.dc_movement = speed_profile

        self.vert_movement1 = rospy.Publisher('front_vert_control', Int32, queue_size=8)
        self.vert_movement2 = rospy.Publisher('rear_vert_control', Int32, queue_size=8)
        rospy.Subscriber('front_tic', Int32, self._stepper1_position)
        rospy.Subscriber('rear_tic', Int32, self._stepper2_position)
        self.dc_pub = dc_motor_pub
        self.vacuum = vacuum
        self.stage1Stop = stopStage1
        self.stage2Stop = stopStage2
        return

    def _stepper1_position(self, data: Int32):
        self.mtx1.acquire()
        self.frontPos = data.data
        self.mtx1.release()
        return

    def _stepper2_position(self, data: Int32):
        self.mtx2.acquire()
        self.rearPos = data.data
        self.mtx2.release()
        return

    def reset(self):
        self.vert_movement1.publish(self.frontTargets['high'])
        self.vert_movement2.publish(self.rearTargets['high'])
        self.vacuum.publish(VACUUM['off'])

    def next(self, readings, up: bool = True) -> bool:
        rospy.loginfo(readings)
        finished = False
        self.mtx1.acquire()
        self.mtx2.acquire()
        if up:
            if self.currentState == self.climbState.CLEAN:
                self.vacuum.publish(VACUUM['off'])
                self.currentState = self.climbState.LIFT_MIDDLE
                self.vert_movement1.publish(self.frontTargets['low'])
                self.vert_movement2.publish(self.rearTargets['low'])
            elif self.currentState == self.climbState.LIFT_MIDDLE:
                if self.frontPos == self.frontTargets['low'].data and self.rearPos == self.rearTargets['low'].data:
                    while self.stage1Stop:
                        pass
                    self.currentState = self.climbState.FORWARD1
                    self.dc_pub.publish(self.dc_movement['climb'])
            elif self.currentState == self.climbState.FORWARD1:
                if not readings[SENSOR_INDEX['rear-right']] and not readings[SENSOR_INDEX['rear-left']]:
                    self.dc_pub.publish(self.dc_movement['stop'])
                    self.currentState = self.climbState.LIFT_ENDS
                    self.vert_movement1.publish(self.frontTargets['high'])
                    self.vert_movement2.publish(self.rearTargets['high'])
            elif self.currentState == self.climbState.LIFT_ENDS:
                if self.frontPos == self.frontTargets['high'].data and self.rearPos == self.rearTargets['high'].data:
                    while self.stage2Stop:
                        pass
                    self.currentState = self.climbState.FORWARD2
                    self.dc_pub.publish(self.dc_movement['climb'])
            elif self.currentState == self.climbState.FORWARD2:
                if not readings[SENSOR_INDEX['center-right']] and not readings[SENSOR_INDEX['center-left']]:
                    self.dc_pub.publish(self.dc_movement['stop'])
                    self.currentState = self.climbState.CLEAN
                    rospy.Rate(1).sleep()
                    self.vacuum.publish(VACUUM['on'])
                    rospy.Rate(1).sleep()
                    finished = True
        else:
            rospy.logerr('Not implemented')
        self.mtx1.release()
        self.mtx2.release()
        return finished


class cleanStateMachine:
    class currentState(enum.Enum):
        INITIALIZATION = 0x0
        CLEAN = 0x1
        STEP = 0x2
        FINISHED = 0x3

    def __init__(self, speed_profile: horizontalSpeeds, up: bool = True, startingLeft: bool = True):
        self.up = up
        self.current_state = self.currentState.STEP

        self.sensor_mtx = threading.Lock()
        self.readings = []

        rospy.Subscriber('ir_sensor', UInt8MultiArray, self._sensors_callback)
        self.vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size=1)
        self.horizontal_movement = rospy.Publisher('horizontal_control', UInt8MultiArray, queue_size=4)

        self.step = stepStateMachine(self.horizontal_movement, speed_profile, self.vacuum_pub, frontL=-16600,
                                     rearL=-16810, frontH=0, rearH=0)
        self.horizontal = HorizontalMovement(self.horizontal_movement, startingLeft, speed_profile, False)

        self._wait_for_subscribers()

    def _wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and (
                self.horizontal_movement.get_num_connections() == 0 or self.step.vert_movement1.get_num_connections()
                == 0 or
                self.step.vert_movement2.get_num_connections() == 0 or self.vacuum_pub.get_num_connections() == 0):
            if i == 4:
                if self.horizontal_movement.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.horizontal_movement.name}')
                if self.step.vert_movement1.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.step.vert_movement1.name}')
                if self.step.vert_movement2.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.step.vert_movement2.name}')
                if self.vacuum_pub.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.vacuum_pub.name}')
            rospy.Rate(10).sleep()
            i += 1
            i %= 5
        if rospy.is_shutdown():
            raise Exception('Got shutdown request before subscribers could connect')
        return

    def _sensors_callback(self, data: UInt8MultiArray):
        self.sensor_mtx.acquire()
        self.readings = data.data
        self.sensor_mtx.release()
        return

    def next(self):
        refreshRate = 1.25
        self.sensor_mtx.acquire()
        readings = copy.deepcopy(self.readings)
        self.sensor_mtx.release()

        if self.current_state == self.currentState.INITIALIZATION:
            self.vacuum_pub.publish(VACUUM['on'])
            self.current_state = self.currentState.CLEAN
        elif self.current_state == self.currentState.CLEAN:
            if self.horizontal.next(readings):
                self.current_state = self.currentState.STEP
                self.horizontal.resetStateMachine()
        elif self.current_state == self.currentState.STEP:
            if self.step.next(readings, self.up):
                # TODO: Figure out when to stop
                self.current_state = self.currentState.CLEAN
            refreshRate = 20

        return refreshRate, self.current_state == self.currentState.FINISHED


def main():
    rospy.init_node('summit_sweeper_main_run', disable_signals=True)
    dc_speeds = horizontalSpeeds('https://raw.githubusercontent.com/24778-TeamB/motor-speeds/master/speeds.json')
    clean = cleanStateMachine(dc_speeds, startingLeft=False)

    finished = False

    try:
        while not rospy.is_shutdown() and not finished:
            refresh, finished = clean.next()
            rospy.Rate(refresh).sleep()
    except KeyboardInterrupt:
        clean.step.reset()
        clean.horizontal.resetStateMachine()
    finally:
        rospy.signal_shutdown('Shutting down node')
    return


if __name__ == '__main__':
    main()
