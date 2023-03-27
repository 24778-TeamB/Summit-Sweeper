import RPi.GPIO as GPIO
from std_msgs.msg import Int8
import rospy


VACUUM_PIN = 10
testMode = False


def initVacuums():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(VACUUM_PIN, GPIO.OUT, initial=GPIO.LOW)
    return


def toggleVacuum(turnOn: bool):
    polarity = GPIO.HIGH if turnOn else GPIO.LOW
    GPIO.output(VACUUM_PIN, polarity)
    return


def callback_listener(msg: Int8):
    global testMode
    data = msg.data
    # 0: Pin low
    # 1: Pin high
    # 2: loop toggle (test purposes only)

    if data < 0 or data > 2:
        rospy.logerr(f'{data} is an invalid input')
        return

    if data < 2:
        if testMode:
            rospy.logerr('Currently in test mode. Please disable test mode')
            return
        toggleVacuum(bool(data))
        return
    testMode = not testMode


def main():
    global testMode
    initVacuums()
    rospy.init_node('summit_sweeper_vacuum_control')
    rospy.Subscriber('vacuum_control_sub', Int8, callback_listener)
    rospy.loginfo('Starting vacuum controller.\nUsage: \n0: Turn off\n1: Turn on\n2: toggle test mode')
    while not rospy.is_shutdown():
        if testMode:
            rospy.Rate(1).sleep()
            rospy.logdebug('Vacuums on')
            toggleVacuum(True)
            rospy.Rate(1).sleep()
            rospy.logdebug('Vacuums off')
            toggleVacuum(False)
        rospy.Rate(10).sleep()


if __name__ == '__main__':
    main()

