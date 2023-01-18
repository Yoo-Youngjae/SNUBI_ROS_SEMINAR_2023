import rospy
from std_msgs.msg import String

def callback(data):
   print("I heard", data.data)

if __name__ == '__main__':
    rospy.init_node('simple_subscriber')
    str_subscriber = rospy.Subscriber("str_send", String, callback)
    print('listen')
    rospy.spin()