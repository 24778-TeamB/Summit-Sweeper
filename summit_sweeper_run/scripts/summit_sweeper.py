import rospy
from std_msgs.msg import Float32MultiArray, Int8, Int32
import threading
import copy
import enum


mtx = threading.Lock()
SENSOR_READINGS = []

DC_MOTOR = {
        'forward': 3,
        'reverse': 4,
        'left': 1,
        'right': 2
        'stop': 0
        }

VACUUM = {
        'on': 1,
        'off': 0,
        'test': 2
        }

CLEAN_STATE = {
        'no-state': -1
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


def sensor_callback(data: Float32MultiArray):
    global mtx
    global SENSOR_READINGS
    mtx.acquire()
    SENSOR_READINGS = data.data
    mtx.release()
    return


def clean_down(horizontal_pub, vertical_pub1, vertical_pub2, vacuum_pub, readings):
    global current_state
    if current_state == CLEAN_STATE['left']:
        pass
    elif current_state == CLEAN_STATE['right']:
        pass
    elif current_state == CLEAN_STATE['step']:
        pass
    elif current_state == CLEAN_STATE['no-state']:
        pass


def clean_up(horizontal_pub, vertical_pub1, vertical_pub2, vacuum_pub, readings):
    global current_state
    if current_state == CLEAN_STATE['left']:
        pass
    elif current_state == CLEAN_STATE['right']:
        pass
    elif current_state == CLEAN_STATE['step']:
        pass
    elif current_state == CLEAN_STATE['no-state']:
        pass


def main():
    global mtx
    global SENSOR_READINGS
    rospy.init_node('summit_sweeper_main_run')
    rospy.Subscriber('ultra_sonic', Float32MultiArray, sensor_callback)
    vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size = 1)
    horizontal_pub = rospy.Publisher('horizontal_control', Int8, queue_size = 8)
    vert_movement1 = rospy.Publisher('front_vert_control', Int32, queue_size = 8)
    vert_movement2 = rospy.Publisher('rear_vert_control', Int32, queue_size = 8)
    wait_for_subscribers(horizontal_pub, vert_movement1, vert_movement2, vacuum_pub)
    state = currentState.INITIALIZATION

    while not rospy.is_shutdown():
        # Copy the readings to prevent blocking
        mtx.acquire()
        readings = copy.deepcopy(SENSOR_READINGS)
        mtx.release()

        if state == currentState.CLEAN_UP:
            clean_up(horizontal_pub, vert_movement1, vert_movement2, vacuum_pub, readings)
            # check if job is finished here
        elif state == currentState.CLEAN_DOWN:
            clean_down(horizontal_pub, vert_movement1, vert_movement2, vacuum_pub, readings)
            # Check if job is finished here
        elif state == currentState.FINISHED:
            vacuum_pub.publish(Int8(data=VACUUM['off']))
            vert_movement1.publish(Int32(data=0))
            vert_movement2.publish(Int32(data=0))
            horizontal_pub.publish(Int8(data=DC_MOTOR['forward']))
            rospy.Rate(1).sleep()
            horizontal_pub.publish(Int8(data=DC_MOTOR['stop']))
            break
        elif state == currentState.INITIALIZATION:
            pass # Decide whether to clean up or down here

        rospy.Rate(100).sleep()
        del readings


if __name__ == '__main__':
    main()
