import rospy
import std_msgs


def callback_listener(dataValues: std_msgs.msg.Float32MultiArray):
    rospy.loginfo(f'{dataValues.data}')
    return

def main():
    global kf
    global mutex
    rospy.init_node('summit_sweeper_distance_sub')
    rospy.Subscriber('ultra_sonic', std_msgs.msg.Float32MultiArray, callback_listener)
    rospy.spin()

if __name__ == '__main__':
    main()

