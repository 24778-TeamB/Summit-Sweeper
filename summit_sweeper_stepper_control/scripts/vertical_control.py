import rospy
import std_msgs
import threading
from ticlib import TicUSB, TIC36v4
from time import sleep
import signal
import logging

mtx1 = threading.Lock()
mtx2 = threading.Lock()

frontTic = TicUSB(product=TIC36v4, serial_number='00414637')
# rearTic = TicUSB(product=TIC36v4, serial_number=None)


# https://stackoverflow.com/a/21919644/11854714
class DelayedKeyboardInterrupt:

    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.signal(signal.SIGINT, self.handler)

    def handler(self, sig, frame):
        self.signal_received = (sig, frame)
        logging.debug('SIGINT received. Delaying KeyboardInterrupt.')

    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)


def callbackSetPos1(pos: std_msgs.msg.Int32):
    global mtx1
    global frontTic
    mtx1.acquire()
    frontTic.set_target_position(pos.data)
    mtx1.release()


def callbackSetPos2(pos: std_msgs.msg.Int32):
    global mtx2
    # global rearTic
    mtx2.acquire()
    # rearTic.set_target_position(pos.data)
    mtx2.release()


def main():
    global mtx1
    global mtx2
    global frontTic
    # global rearTic

    # Initialize TICs
    frontTic.halt_and_set_position(0)
    frontTic.energize()
    frontTic.exit_safe_start()

    # rearTic.halt_and_set_position(0)
    # rearTic.energize()
    # rearTic.exit_safe_start()

    rospy.init_node('summit_sweeper_vertical_control')
    frontPub = rospy.Publisher('front-tic', std_msgs.msg.Int32, queue_size=4)
    rearPub = rospy.Publisher('rear-tic', std_msgs.msg.Int32, queue_size=4)
    rospy.Subscriber('front-vert-control', std_msgs.msg.Int32, callbackSetPos1)
    rospy.Subscriber('rear-vert-control', std_msgs.msg.Int32, callbackSetPos2)
    rospy.loginfo('Starting vertical control')

    while not rospy.is_shutdown():
        try:
            mtx1.acquire()
            frontPos = frontTic.get_current_position()
            mtx1.release()
            mtx2.acquire()
            rearPos = 0  # rearTic.get_current_position()
            mtx2.release()

            front = std_msgs.msg.Int32()
            rear = std_msgs.msg.Int32()

            if frontPos is not None:
                front.data = frontPos
                frontPub.publish(front)
            if rearPos is not None:
                rear.data = rearPos
                rearPub.publish(rear)
            
            rospy.Rate(10).sleep()
        except KeyboardInterrupt:
            print('Keyboard interrupt received. Resetting stepper motors to their original position.')
            break
        except Exception as e:
            print(str(e))

    # Cleanup
    with DelayedKeyboardInterrupt():
        mtx1.acquire()  # Lock mutex just in case
        print('Resetting stepper motors')
        frontTic.set_target_position(0)
        while frontTic.get_current_position() != frontTic.get_target_position():
            sleep(0.1)

        frontTic.deenergize()
        frontTic.enter_safe_start()
        mtx1.release()

        mtx2.acquire()
        # rearTic.set_target_position(0)
        # while rearTic.get_current_position() != rearTic.get_target_position():
        #     sleep(0.1)

        # rearTic.deenergize()
        # rearTic.enter_safe_start()
        mtx2.release()


if __name__ == '__main__':
    main()

