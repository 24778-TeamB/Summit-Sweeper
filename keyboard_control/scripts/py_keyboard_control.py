# Based off of: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
#!/usr/bin/env python

from __future__ import print_function

import threading
import rospy
import sys
from select import select
from std_msgs.msg import Int8, Int32

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Horizontal Movement:
W -> Forward
S -> Reverse
A -> Left
D -> Right

Vertical Movement:
R -> Front Up
F -> Front Down
T -> Rear Up
G -> Rear Down
Y -> All Up
H -> All Down

Vacuum Control:
V -> On
B -> Off

Press nothing -> stop

CTRL-C to quit
"""

moveBindings = {
    'w': 3,
    's': 4,
    'a': 1,
    'd': 2
}

frontBindings = {
    'r': 100,
    'f': -100
}

rearBindings = {
    't': 100,
    'g': -100
}

stepperAllBindings = {
    'y': 100,
    'h': -100
}

vacuumBindings = {
        'b': 0,
        'v': 1
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('horizontal_control', Int8, queue_size = 1)
        self.vacuum_publisher = rospy.Publisher('vacuum_control_sub', Int8, queue_size = 1)
        # rospy.Subscriber('front_target', Int32, None)
        # rospy.Subscriber('rear_target', Int32, None)
        self.frontPub = rospy.Publisher('front_vert_control', Int32, queue_size = 4)
        self.rearPub = rospy.Publisher('rear_vert_control', Int32, queue_size = 4)
        self.state = 0
        self.condition = threading.Condition()
        self.done = False
        self.vacuum_state = 0
        self.frontTarget = 0
        self.rearTarget = 0

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and (self.publisher.get_num_connections() == 0 or self.vacuum_publisher.get_num_connections() == 0):
            if i == 4 and self.publisher.get_num_connections() == 0:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            if i == 4 and self.vacuum_publisher.get_num_connections() == 0:
                print("Wating for subscriber to connect to {}".format(self.vacuum_publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, movement, vacuum = None, front = None, rear = None):
        self.condition.acquire()
        self.state = movement
        if vacuum is not None:
            self.vacuum_state = vacuum
        if front is not None:
            self.frontTarget += front
            if self.frontTarget < 0:
                self.frontTarget = 0
        if rear is not None:
            self.rearTarget += rear
            if self.rearTarget < 0:
                self.rearTarget = 0
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        pubValue = Int8()
        vacValue = Int8()
        frontValue = Int32()
        rearValue = Int32()
        pubValue.data = 0
        vacValue.data = 0
        prevData = 1
        prevVac = 1
        prevFront = 1
        prevRear = 1
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            pubValue.data = self.state
            vacValue.data = self.vacuum_state
            frontValue.data = self.frontTarget
            rearValue.data = self.rearTarget

            self.condition.release()

            # Publish.
            if prevData != pubValue.data:
                self.publisher.publish(pubValue)
            if prevVac != vacValue.data:
                self.vacuum_publisher.publish(vacValue)
            if prevFront != frontValue.data:
                self.frontPub.publish(frontValue)
            if prevRear != rearValue.data:
                self.rearPub.publish(rearValue)
            prevData = pubValue.data
            prevVac = vacValue.data
            prevFront = frontValue.data
            prevRear = rearValue.data

        pubValue.data = 0
        vacValue.data = 0
        # Publish stop message when thread exits.
        self.publisher.publish(pubValue)
        self.vacuum_publisher.publish(vacValue)


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

    rospy.init_node('summit_climber_teleop')

    pub_thread = PublishThread(1)

    movement = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(movement)

        print(msg)
        while(1):
            vacuumEn = None
            frontUpdate = None
            rearUpdate = None
            key = getKey(settings, 1)
            if key in moveBindings.keys():
                movement = moveBindings[key]
            elif key in vacuumBindings.keys():
                vacuumEn = vacuumBindings[key]
            elif key in frontBindings.keys():
                frontUpdate = frontBindings[key]
            elif key in rearBindings.keys():
                rearUpdate = rearBindings[key]
            elif key in stepperAllBindings.keys():
                frontUpdate = stepperAllBindings[key]
                rearUpdate = frontUpdate
            elif movement != 0 and key == '':
                movement = 0
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and movement == 0:
                    continue
                if (key == '\x03'):
                    break

            pub_thread.update(movement, vacuumEn, frontUpdate, rearUpdate)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
