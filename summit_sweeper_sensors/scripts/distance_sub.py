import rospy
import std_msgs
import threading

kf = None
mutex = threading.Lock()


def callback_listener(dataValues: std_msgs.msg.Float32Array):
    global kf
    global mutex
    mutex.acquire()
    newData = dataValues.data
    # TODO: input into Kalman filter
    threading.release()
    return

def wait_for_subscribers(pub: rospy.Publisher):
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        rospy.loginfo(f'Wating for subscriber to connect to {pub.name}')
        rospy.Rate(4).sleep()
    if rospy.is_shutdown():
        raise Exception("Got shutdown request before subscribers connected")
    return

def main():
    global kf
    global mutex
    rospy.init_node('summit_sweeper_distance_filter')
    # TODO: Initialize Kalman Filter
    rospy.Subscriber('ultra-sonic', std_msgs.msg.Float32Array, callback_listener)
    pub = rospy.Publisher('ultra-sonic-filtered', std_msgs.msg.Float32Array)
    wait_for_subscribers(pub)
    rospy.loginfo('Starting ultra-sonic filter')
    while not rospy.is_shutdown():
        mutex.acquire()
        data = []  # TODO: Get filtered values
        mutex.release()
        pub_msg = std_msgs.msg.Float32Array(data=data)
        pub.publish(pub_msg)
