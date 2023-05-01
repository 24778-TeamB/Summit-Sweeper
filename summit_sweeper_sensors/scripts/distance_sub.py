import rospy
import std_msgs


def callback_listener(dataValues: std_msgs.msg.UInt8MultiArray):
    rospy.loginfo(f'{dataValues.data}')
    return


def main():
    rospy.init_node('summit_sweeper_distance_sub')
    rospy.Subscriber('ir_sensor', std_msgs.msg.UInt8MultiArray, callback_listener)
    rospy.spin()


if __name__ == '__main__':
    main()
