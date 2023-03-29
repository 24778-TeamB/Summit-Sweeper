import rospy
from std_msgs.msg import Int8
import serial

port = None

MOVEMENT = {
        0: 'stop',
        1: 'left',
        2: 'right',
        3: 'forward',
        4: 'reverse'
        }


def issueMovement(msg: Int8):
    global port
    movement = msg.data
    try:
        rospy.loginfo(f'Moving {MOVEMENT[movement]}')
    except KeyError:
        rospy.logerr(f'Invalid movement command: {movement} does to correlate with any of the valid movements')
        return

    try:
        if port is None:
            rospy.logerr('Unable to write to motor controller: port is closed!')
            return
        port.write(f'motor {MOVEMENT[movement]}\r\n'.encode('UTF-8'))
    except serial.SerialTimeoutException:
        rospy.logerr('Horizontal Motor Controller Timed out!')
    return


def main():
    global port
    port = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    rospy.init_node('summit_sweeper_horizontal_movement')
    rospy.Subscriber('horizontal_control', Int8, issueMovement)
    rospy.loginfo('Starting horizontal control')
    rospy.spin()
    return


if __name__ == '__main__':
    main()

