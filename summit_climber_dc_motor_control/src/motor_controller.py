import rospy
from std_msgs.msg import Int8
import serial


MOVEMENT = {
        0: 'stop',
        1: 'left',
        2: 'right',
        3: 'forward',
        4: 'backwards'
        }


def issueMovement(msg: Int8):
    movement = msg.data
    try:
        rospy.loginfo(f'Moving {MOVEMENT[movement]}')
        # TODO: Add serial port stuff
    except KeyError:
        rospy.logerr(f'Invalid movement command: {movement} does to correlate with any of the valid movements')
    return


def main():
    rospy.init_node('summit_climber_horizontal_movement')
    rospy.Subscriber('horizontal_control', Int8, issueMovement)
    rospy.loginfo('Starting horizontal control')
    rospy.spin()
    return


main()

