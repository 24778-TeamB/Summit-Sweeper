import rospy
import serial
import std_msgs

def main():
    port = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    rospy.init_node('summit_sweeper_distance_sensors')
    pub = rospy.Publisher('ultra_sonic', std_msgs.msg.Float32MultiArray, queue_size=8)
    waitBuf = rospy.Rate(1000)
    while not rospy.is_shutdown():
        port.write(b'dist left\r\n')
        leftData = port.readline()
        waitBuf.sleep()
        port.write(b'dist right\r\n')
        rightData = port.readline()
        waitBuf.sleep()
        port.write(b'dist forward\r\n')
        port.readline()
        waitBuf.sleep()
        port.write(b'dist down\r\n')
        downData = port.readline()
        waitBuf.sleep()
        data = leftData.decode('UTF-8')[:-2].split(', ') + rightData.decode('UTF-8')[:-2].split(', ') + forwardData.decode('UTF-8')[:-2].split(', ') + downData.decode('UTF-8')[:-2].split(', ')
        try:
            data = [float(d) for d in data]
            pub_msg = std_msgs.msg.Float32MultiArray(data=data)
            pub.publish(pub_msg)
        except ValueError:
            rospy.logerr('Tried publishing non-float value. Skipping this publish. If this continues to happen, '
                         'please check Arduino.')
        rospy.Rate(10).sleep()



if __name__ == '__main__':
    main()

