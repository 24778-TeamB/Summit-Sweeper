import rospy
from std_msgs.msg import UInt8MultiArray, Int8, Int32
import threading
import copy
import enum


mtx = threading.Lock()
SENSOR_READINGS = []

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


current_state = CLEAN_STATE['no-state']


class currentState(enum.Enum):
    INITIALIZATION = 0x0
    CLEAN_UP = 0x1
    CLEAN_DOWN = 0x2
    FINISHED = 0x3


class stepStateMachine:
    class climbState(enum.Enum):
        CLEAN = 0X0
        LIFT_MIDDLE = 0X1
        FORWARD1 = 0X2
        LIFT_FRONT = 0X3
        LIFT_REAR = 0X4
        FORWARD2 = 0X5

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
        self.currentState: self.climbState = self.climbState.CLEAN

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
                if True:  # TODO: check sensor readings here
                    self.dc_pub.publish(Int8(data=DC_MOTOR['stop']))
                    self.currentState = self.climbState.LIFT_FRONT
                    self.vert_movement1.publish(self.frontTargets['high'])
                else:
                    self.dc_pub.publish(Int8(data=DC_MOTOR['forward']))
            elif self.currentState == self.climbState.LIFT_FRONT:
                if self.frontPos == self.frontTargets['high'].data:
                    self.currentState = self.climbState.LIFT_REAR
                    self.vert_movement2.publish(self.rearTargets['high'])
            elif self.currentState == self.climbState.LIFT_REAR:
                if self.rearPos == self.rearTargets['high'].data:
                    self.climbState.FORWARD2
            elif self.currentState == self.climbState.FORWARD2:
                if True:  # TODO: check sensor
                    self.dc_pub.publish(Int8(data=DC_MOTOR['stop']))
                    self.currentState = self.climbState.CLEAN
                    self.vacuum.publish(Int8(data=VACUUM['on']))
                    finished = True
                else:
                    self.dc_pub.publish(Int8(data=DC_MOTOR['forward']))
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


def sensor_callback(data: UInt8MultiArray):
    global mtx
    global SENSOR_READINGS
    mtx.acquire()
    SENSOR_READINGS = data.data
    mtx.release()
    return


def clean_down(horizontal_pub, step_state_machine, vacuum_pub, readings):
    global current_state
    if current_state == CLEAN_STATE['left']:
        pass
    elif current_state == CLEAN_STATE['right']:
        pass
    elif current_state == CLEAN_STATE['step']:
        done = step_state_machine.next(readings, False)
        # TODO check done
    elif current_state == CLEAN_STATE['no-state']:
        pass


def clean_up(horizontal_pub, step_state_machine, vacuum_pub, readings):
    global current_state
    current_state = CLEAN_STATE['step']
    if current_state == CLEAN_STATE['left']:
        pass
    elif current_state == CLEAN_STATE['right']:
        pass
    elif current_state == CLEAN_STATE['step']:
        done = step_state_machine.next(readings, True)
        # TODO check done
        if done:
            exit()
    elif current_state == CLEAN_STATE['no-state']:
        pass


def main():
    global mtx
    global SENSOR_READINGS
    rospy.init_node('summit_sweeper_main_run')
    rospy.Subscriber('ir_sensors', UInt8MultiArray, sensor_callback)
    vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size = 1)
    horizontal_pub = rospy.Publisher('horizontal_control', Int8, queue_size = 8)
    steps = stepStateMachine(horizontal_pub, frontL = -16700, rearL = -16700, frontH=0, rearH=0)
    wait_for_subscribers(horizontal_pub, steps.vert_movement1, steps.vert_movement2, vacuum_pub)
    state = currentState.INITIALIZATION

    while not rospy.is_shutdown():
        # Copy the readings to prevent blocking
        mtx.acquire()
        readings = copy.deepcopy(SENSOR_READINGS)
        mtx.release()
        s_mtx1.acquire()
        front = step1_pos
        s_mtx1.release()
        s_mtx2.acquire()
        rear = step2_pos
        s_mtx2.release()

        if state == currentState.CLEAN_UP:
            clean_up(horizontal_pub, vert_movement1, vert_movement2, vacuum_pub, readings, front, rear)
            # check if job is finished here
        elif state == currentState.CLEAN_DOWN:
            clean_down(horizontal_pub, vert_movement1, vert_movement2, vacuum_pub, readings, front, rear)
            # Check if job is finished here
        elif state == currentState.FINISHED:
            vacuum_pub.publish(Int8(data=VACUUM['off']))
            vert_movement1.publish(Int32(data=0)) # Initialize one of these to -15170
            vert_movement2.publish(Int32(data=0))
            horizontal_pub.publish(Int8(data=DC_MOTOR['forward']))
            rospy.Rate(1).sleep()
            horizontal_pub.publish(Int8(data=DC_MOTOR['stop']))
            break
        elif state == currentState.INITIALIZATION:
            state = currentState.CLEAN_UP
            #pass # Decide whether to clean up or down here

        rospy.Rate(100).sleep()
        del readings


if __name__ == '__main__':
    main()
