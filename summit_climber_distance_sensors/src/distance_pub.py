import rospy
import serial
import std_msgs

def main():
    port = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    rospy.init_node('summit_sweeper_distance_sensors')
    pub = rospy.Publisher('topic-name', std_msgs.msg.Float32MultiArray, queue_size=8)
    randArray = [0.89, 0.95, 1.3, 1.5]
    pub_msg = std_msgs.msg.Float32MultiArray(data=randArray)
    pub.publish(pub_msg)
    while not rospy.is_shutdown():
        pass

