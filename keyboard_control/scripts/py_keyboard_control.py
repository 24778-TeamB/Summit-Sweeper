#!/usr/bin/env python

from __future__ import print_function

import threading
import rospy
import sys
from select import select
from std_msgs.msg import Int8

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
W -> Forward
S -> Reverse
A -> Left
D -> Right
anything else -> stop

CTRL-C to quit
"""

moveBindings = {
    'w': 3,
    's': 4,
    'a': 1,
    'd': 2
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('horizontal_control', Int8, queue_size = 1)
        self.state = 0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, movement):
        self.condition.acquire()
        self.state = movement
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        pubValue = Int8()
        pubValue.data = 0
        prevData = 1
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            pubValue.data = self.state

            self.condition.release()

            # Publish.
            if prevData != pubValue.data:
                self.publisher.publish(pubValue)
            prevData = pubValue.data

        pubValue.data = 0
        # Publish stop message when thread exits.
        self.publisher.publish(pubValue)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    pub_thread = PublishThread(1)

    movement = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(movement)

        print(msg)
        while(1):
            key = getKey(settings, 1)
            if key in moveBindings.keys():
                movement = moveBindings[key]
            elif movement != 0 and key == '':
                movement = 0
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and movement == 0:
                    continue
                if (key == '\x03'):
                    break

            pub_thread.update(movement)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
