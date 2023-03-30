import rospy
import serial
import std_msgs

def main():
    port = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    rospy.init_node('summit_sweeper_distance_sensors')
    pub = rospy.Publisher('ultra-sonic', std_msgs.msg.Float32MultiArray, queue_size=8)
    pub_msg = std_msgs.msg.Float32MultiArray(data=randArray)
    pub.publish(pub_msg)
    waitBuf = rospy.Rate(1000)
    while not rospy.is_shutdown():
        port.write('dist left\r\n'.encode('UTF-8'))
        leftData = port.read_until()
        waitBuf.sleep()
        port.reset_input_buffer()
        port.write('dist right\r\n'.encode('UTF-8'))
        rightData = port.read_until()
        waitBuf.sleep()
        port.reset_input_buffer()
        port.write('dist forward\r\n'.encode('UTF-8'))
        port.read_until()
        waitBuf.sleep()
        forwardData = port.reset_input_buffer()
        port.write('dist down\r\n'.encode('UTF-8'))
        downData = port.read_until()
        waitBuf.sleep()
        port.reset_input_buffer()
        data = leftData.decode('UTF-8').split(', ') + rightData.decode('UTF-8').split(', ') + forwardData.decode(
            'UTF-8').split(', ') + downData.decode('UTF-8').split(', ')
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

