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

FRONT_RIGHT_SPEEDS = {
    'right': 0,
    'left': 0,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

FRONT_LEFT_SPEEDS = {
    'right': 0,
    'left': 0,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

MID_RIGHT_SPEEDS = {
    'right': 225,
    'left': 180,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

MID_LEFT_SPEEDS = {
    'right': 120,
    'left': 250,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

REAR_RIGHT_SPEEDS = {
    'right': 120,
    'left': 250,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

REAR_LEFT_SPEEDS = {
    'right': 250,
    'left': 180,
    'forward': 100,
    'reverse': 100,
    'stop': 0
}

FRONT_RIGHT_SPEED = 125
FRONT_LEFT_SPEED = 125
MID_RIGHT_SPEED = 125
MID_LEFT_SPEED = 125
REAR_RIGHT_SPEED = 125
REAR_LEFT_SPEED = 125


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
        port.write(f'set speed r1 {MID_RIGHT_SPEED[MOVEMENT[movement]]}')
        port.write(f'set speed r2 {REAR_RIGHT_SPEED[MOVEMENT[movement]]}')
        port.write(f'set speed r3 {FRONT_RIGHT_SPEED[MOVEMENT[movement]]}')
        port.write(f'set speed l1 {MID_LEFT_SPEED[MOVEMENT[movement]]}')
        port.write(f'set speed l2 {REAR_LEFT_SPEED[MOVEMENT[movement]]}')
        port.write(f'set speed l3 {FRONT_LEFT_SPEED[MOVEMENT[movement]]}')
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
    port = serial.Serial(port_name, baud)
    rospy.loginfo('Calibrating speeds')
    port.write(f'set speed r1 {MID_RIGHT_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed r2 {REAR_RIGHT_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed r3 {FRONT_RIGHT_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l1 {MID_LEFT_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l2 {REAR_LEFT_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l3 {FRONT_LEFT_SPEED}\r\n'.encode('UTF-8'))

    rospy.Subscriber('horizontal_control', Int8, issueMovement)
    rospy.loginfo('Starting horizontal control')
    rospy.spin()
    return


if __name__ == '__main__':
    main()

