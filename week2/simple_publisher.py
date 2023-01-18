import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('simple_publisher')
    str_publisher = rospy.Publisher('str_send', String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        send_str = input('string to send : ')
        str_publisher.publish(send_str)
        rate.sleep()
        if send_str == 'q':
            break