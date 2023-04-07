import rospy
from std_msgs.msg import Float32MultiArray, Int8, Int32
import threading


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


def main():
    global mtx
    global SENSOR_READINGS
    rospy.init_node('summit_sweeper_main_run')
    rospy.Subscriber('ultra_sonic', Float32MultiArray, sensor_callback)
    vacuum_pub = rospy.Publisher('vacuum_control_sub', Int8, queue_size = 1)


