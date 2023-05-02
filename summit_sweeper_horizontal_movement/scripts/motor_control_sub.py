import rospy
from std_msgs.msg import Int8
import serial
from typing import Optional
import sys

port: Optional[serial.Serial] = None
lastState = 0

MOVEMENT = {
        0: 'motor stop',
        1: 'motor left',
        2: 'motor right',
        3: 'motor forward',
        4: 'motor reverse',
        5: 'rotate cw',
        6: 'rotate ccw'
        }

FRONT_RIGHT_SPEEDS = {
    'motor right': 0,
    'motor left': 0,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 0,
    'rotate ccw': 0
}

FRONT_LEFT_SPEEDS = {
    'motor right': 0,
    'motor left': 0,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 0,
    'rotate ccw': 0
}

MID_RIGHT_SPEEDS = {
    'motor right': 255,
    'motor left': 200,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 0,
    'rotate ccw': 255,
}

MID_LEFT_SPEEDS = {
    'motor right': 200,
    'motor left': 255,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 255,
    'rotate ccw': 0
}

REAR_RIGHT_SPEEDS = {
    'motor right': 200,
    'motor left': 255,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 0,
    'rotate ccw': 255
}

REAR_LEFT_SPEEDS = {
    'motor right': 255,
    'motor left': 200,
    'motor forward': 255,
    'motor reverse': 255,
    'motor stop': 0,
    'rotate cw': 255,
    'rotate ccw': 0
}

INITIAL_SPEED = 128


def issueMovement(msg: Int8):
    global port
    global lastState
    movement = msg.data
    try:
        rospy.loginfo(MOVEMENT[movement])
    except KeyError:
        rospy.logerr(f'Invalid movement command: {movement} does to correlate with any of the valid movements')
        return

    try:
        if port is None:
            rospy.logerr('Unable to write to motor controller: port is closed!')
            return
        port.write(f'set speed r1 {MID_RIGHT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        port.write(f'set speed r2 {REAR_RIGHT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        port.write(f'set speed r3 {FRONT_RIGHT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        port.write(f'set speed l1 {MID_LEFT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        port.write(f'set speed l2 {MID_LEFT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        port.write(f'set speed l3 {MID_LEFT_SPEEDS[MOVEMENT[movement]]}\r\n'.encode('UTF-8'))
        if MOVEMENT[movement].startswith('motor'):
            port.write(f'{MOVEMENT[movement]}\r\n'.encode('UTF-8'))
        else:
            port.write(b'motor forward\r\n')
    except serial.SerialTimeoutException:
        rospy.logerr('Horizontal Motor Controller Timed out!')
    except ValueError:
        rospy.logerr('Bad serial object')
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
    port.write(f'set speed r1 {INITIAL_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed r2 {INITIAL_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed r3 {INITIAL_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l1 {INITIAL_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l2 {INITIAL_SPEED}\r\n'.encode('UTF-8'))
    port.write(f'set speed l3 {INITIAL_SPEED}\r\n'.encode('UTF-8'))

    rospy.Subscriber('horizontal_control', Int8, issueMovement)
    rospy.loginfo('Starting horizontal control')
    rospy.spin()
    port.write(b'motor motor stop\r\n')
    return


if __name__ == '__main__':
    main()

