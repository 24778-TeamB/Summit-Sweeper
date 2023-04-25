import rospy
from std_msgs.msg import UInt8MultiArray, Int8, Int32
import threading
import copy
import enum


DC_MOTOR = {
        'forward': Int8(data=3),
        'reverse': Int8(data=4),
        'left': Int8(data=1),
        'right': Int8(data=2),
        'stop': Int8(data=0)
        }

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
            'side-left': 5,
            'side-right': 4
        }


class stepStateMachine:
    class climbState(enum.Enum):
        CLEAN = 0X0
        LIFT_MIDDLE = 0X1
        FORWARD1 = 0X2
        LIFT_ENDS = 0x3
        FORWARD2 = 0X4

    def __init__(self, dc_motor_pub, vacuum, frontL = 0, rearL = 0, frontH = 1, rearH = 1):
        self.frontTargets = {}
        self.rearTargets = {}
        self.frontTargets['low'] = Int32(data=frontL)
        self.frontTargets['high'] = Int32(data=frontH)
        self.rearTargets['low'] = Int32(data=rearL)
        self.rearTargets['high'] = Int32(data=rearH)
        self.mtx1 = threading.Lock()
        self.mtx2 = threading.Lock()
        self.frontPos = 0
        self.rearPos = 0
        self.currentState = self.climbState.CLEAN

        self.vert_movement1 = rospy.Publisher('front_vert_control', Int32, queue_size=8)
        self.vert_movement2 = rospy.Publisher('rear_vert_control', Int32, queue_size=8)
        rospy.Subscriber('front_tic', Int32, self._stepper1_position)
        rospy.Subscriber('rear_tic', Int32, self._stepper2_position)
        self.dc_pub = dc_motor_pub
        self.vacuum = vacuum
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

    def next(self, readings, up: bool = True) -> bool:
        finished = False
        self.mtx1.acquire()
        self.mtx2.acquire()
        if up:
            if self.currentState == self.climbState.CLEAN:
                self.vacuum.publish(Int8(data=VACUUM['off']))
                self.currentState = self.climbState.LIFT_MIDDLE
                self.vert_movement1.publish(self.frontTargets['low'])
                self.vert_movement2.publish(self.rearTargets['low'])
            elif self.currentState == self.climbState.LIFT_MIDDLE:
                if self.frontPos == self.frontTargets['low'].data and self.rearPos == self.rearTargets['low'].data:
                    self.currentState = self.climbState.FORWARD1
            elif self.currentState == self.climbState.FORWARD1:
                if readings[SENSOR_INDEX['rear-right']] and readings[SENSOR_INDEX['rear-left']]:
                    self.dc_pub.publish(DC_MOTOR['stop'])
                    self.currentState = self.climbState.LIFT_ENDS
                    self.vert_movement1.publish(self.frontTargets['high'])
                    self.vert_movement2.publish(self.rearTargets['high'])
                else:
                    self.dc_pub.publish(DC_MOTOR['forward'])
            elif self.currentState == self.climbState.LIFT_ENDS:
                if self.frontPos == self.frontTargets['high'].data and self.rearPos == self.rearTargets['high'].data:
                    self.currentState = self.climbState.FORWARD2
            elif self.currentState == self.climbState.FORWARD2:
                if readings[SENSOR_INDEX['center-right']] and readings[SENSOR_INDEX['center-left']]:
                    self.dc_pub.publish(DC_MOTOR['stop'])
                    self.currentState = self.climbState.CLEAN
                    self.vacuum.publish(VACUUM['on'])
                    finished = True
                else:
                    self.dc_pub.publish(DC_MOTOR['forward'])
        else:
            rospy.logerr('Not implemented')
        self.mtx1.release()
        self.mtx2.release()
        return finished


class cleanStateMachine:
    class currentState(enum.Enum):
        INITIALIZATION = 0x0
        CLEAN_LEFT = 0x1
        CLEAN_RIGHT = 0x2
        STEP = 0x3
        FINISHED = 0x4

    def __init__(self, up: bool = True):
        self.up = up
        self.current_state = self.currentState.INITIALIZATION
        self.lastRun = self.currentState.CLEAN_LEFT

        self.sensor_mtx = threading.Lock()
        self.readings = []

        rospy.Subscriber('ir_sensor', UInt8MultiArray, self._sensors_callback)
        self.vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size=1)
        self.horizontal_movement = rospy.Publisher('horizontal_control', Int8, queue_size=4)

        self.step = stepStateMachine(self.vacuum_pub, self.horizontal_movement, frontL = -16700, rearL = -16700,
                                     frontH=0, rearH=0)

        self._wait_for_subscribers()

    def _wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and (
                self.horizontal_pub.get_num_connections() == 0 or self.vertical_pub1.get_num_connections() == 0 or
                self.vertical_pub2.get_num_connections() == 0 or self.vacuum_pub.get_num_connections() == 0):
            if i == 4:
                if self.horizontal_pub.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.horizontal_pub.name}')
                if self.vertical_pub1.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.vertical_pub1.name}')
                if self.vertical_pub2.get_num_connections() == 0:
                    rospy.loginfo(f'Waiting for subscriber to connect to {self.vertical_pub2.name}')
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
        self.sensor_mtx.acquire()
        readings = copy.deepcopy(self.readings)
        self.sensor_mtx.release()

        if self.current_state == self.currentState.INITIALIZATION:
            if readings[SENSOR_INDEX['']]:
            self.current_state = self.currentState.CLEAN_LEFT  # TODO figure out what goes here
        if self.current_state == self.currentState.CLEAN_LEFT:
            if not readings[SENSOR_INDEX['center-left']] or not readings[SENSOR_INDEX['center-right']]:
                pass  # TODO: rotate or move forward
            if readings[self.SENSOR_INDEX['side-left']]:
                self.horizontal_movement.publish(DC_MOTOR['stop'])
                self.current_state = self.currentState.STEP
        if self.current_state == self.currentState.CLEAN_RIGHT:
            if not readings[SENSOR_INDEX['center-left']] or not readings[SENSOR_INDEX['center-right']]:
                pass  # TODO: rotate or move forward
            if readings[SENSOR_INDEX['side-right']]:
                self.horizontal_movement.publish(DC_MOTOR['stop'])
                self.current_state = self.currentState.STEP
        if self.current_state == self.currentState.STEP:
            done = self.step.next(readings, self.up)
            if done:
                if self.lastRun == self.currentState.CLEAN_LEFT:
                    self.current_state = self.currentState.CLEAN_RIGHT
                    self.horizontal_movement.publish(DC_MOTOR['right'])
                else:
                    self.current_state = self.currentState.CLEAN_LEFT
                    self.horizontal_movement.publish(DC_MOTOR['left'])
                self.lastRun = self.currentState
                # TODO: Figure out when finished
        return self.current_state == self.currentState.FINISHED


def main():
    rospy.init_node('summit_sweeper_main_run')
    clean = cleanStateMachine()

    finished = False

    while not rospy.is_shutdown() and not finished:
        finished = clean.next()
        rospy.Rate(100).sleep()

    rospy.loginfo('Summit Sweeper done cleaning')
    rospy.spin()


if __name__ == '__main__':
    main()
