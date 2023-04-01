import rospy
import std_msgs


def callback_listener(dataValues: std_msgs.msg.Float32Array):
    rospy.loginfo(f'{dataValues.data}')
    return

def main():
    global kf
    global mutex
    rospy.init_node('summit_sweeper_distance_sub')
    rospy.Subscriber('ultra-sonic', std_msgs.msg.Float32Array, callback_listener)
    rospy.spin()
