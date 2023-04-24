import rospy
from std_msgs.msg import Int32, Int8, Float32MultiArray, UInt8MultiArray
import enum
import threading
import time

DC_MOTOR = {
        'forward': 3,
        'reverse': 4,
        'left': 1,
        'right': 2,
        'stop': 0
        }

VACUUM = {
        'on': 1,
        'off': 0,
        'test': 2
        }

CLEAN_STATE = {
        'no-state': -1,
        'left': 0,
        'right': 1,
        'step': 2
        }

SENSOR_INDEX = {
            'center-left': 1,
            'center-right': 0,
            'rear-left': 3,
            'rear-right': 2,
            'side-left': 5,
            'side-right': 4
        }

READINGS = []

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


def wait_for_subscribers(horizontal_pub, vertical_pub1, vertical_pub2, vacuum_pub):
    i = 0
    while not rospy.is_shutdown() and (horizontal_pub.get_num_connections() == 0 or vertical_pub1.get_num_connections() == 0 or vertical_pub2.get_num_connections() == 0 or vacuum_pub.get_num_connections() == 0):
        if i == 4:
            if horizontal_pub.get_num_connections() == 0:
                rospy.loginfo(f'Waiting for subscriber to connect to {horizontal_pub.name}')
                rospy.loginfo(f'Waiting for subscriber to connect to {vertical_pub1.name}')
                rospy.loginfo(f'Waiting for subscriber to connect to {vertical_pub2.name}')
                rospy.loginfo(f'Waiting for subscriber to connect to {vacuum_pub.name}')
        rospy.Rate(10).sleep()
        i += 1
        i %= 5
    if rospy.is_shutdown():
        raise Exception('Got shutdown request before subscribers could connect')
    return


def _sensors_callback(data: UInt8MultiArray):
    global READINGS
    #sensor_mtx.acquire()
    READINGS = data.data
    #sensor_mtx.release()
    return


def main():
    rospy.init_node('step_test')
    vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size = 1)
    horizontal_pub = rospy.Publisher('horizontal_control', Int8, queue_size = 8)
    rospy.Subscriber('ir_sensors', UInt8MultiArray, self._sensors_callback)
    step = stepStateMachine(horizontal_pub, vacuum_pub, frontL = -16700, rearL = -16700, frontH=0, rearH=0)
    wait_for_subscribers(horizontal_pub, step.vert_movement1, step.vert_movement2, vacuum_pub)
    done = False
    while not done and not rospy.is_shutdown():
        done = step.next(READINGS)
        rospy.Rate(10).sleep()
    rospy.spin()


if __name__ == '__main__':
    main()

