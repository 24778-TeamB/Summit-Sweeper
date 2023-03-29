import rospy
import serial
import std_msgs

def main():
    port = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    rospy.init_node('summit_sweeper_distance_sensors')
    pub = rospy.Publisher('ultra-sonic', std_msgs.msg.Float32MultiArray, queue_size=8)
    pub_msg = std_msgs.msg.Float32MultiArray(data=randArray)
    pub.publish(pub_msg)
    while not rospy.is_shutdown():
        port.write('dist left\r\n'.encode('UTF-8'))
        # Read data here
        port.write('dist right\r\n'.encode('UTF-8'))
        # Read and append data
        port.write('dist forward\r\n'.encode('UTF-8'))
        # Read and append data
        port.write('dist down\r\n'.encode('UTF-8'))
        # Read and append data
        data = []  # TODO: convert data into list of floats
        pub_msg = std_msgs.msg.Float32MultiArray(data=data)
        pub.publish(pub_msg)
        rospy.Rate(10).sleep()



if __name__ == '__main__':
    main()

