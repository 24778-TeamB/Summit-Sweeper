import rospy
from std_msgs.msg import Int8
import serial
import sys

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
    rospy.init_node('summit_sweeper_horizontal_movement')

    port_name = rospy.get_param('~port', '/dev/ttyACM0')
    baud = int(rospy.get_param('~baud', '9600'))

    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2:
        port_name = sys.argv[1]

    rospy.loginfo(f'Connecting to {port_name} with a baud of {baud}')
    port = serial.Serial(port_name, baud, timeout=5)

    rospy.Subscriber('horizontal_control', Int8, issueMovement)
    rospy.loginfo('Starting horizontal control')
    rospy.spin()
    return


if __name__ == '__main__':
    main()

