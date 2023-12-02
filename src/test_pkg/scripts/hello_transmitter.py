import rospy
from std_msgs.msg import String  # Import ROS String message type

def hello_publisher():
    rospy.init_node('hello_publisher', anonymous=True)
    hello_pub = rospy.Publisher('hello_topic', String, queue_size=10)  # Create a publisher for 
'hello_topic'

    rate = rospy.Rate(1)  # 1 Hz (transmit "hello" every second)

    while not rospy.is_shutdown():
        hello_pub.publish("hello")  # Publish the string "hello" to the 'hello_topic'
        rate.sleep()

if __name__ == '__main__':
    try:
        hello_publisher()
    except rospy.ROSInterruptException:
        pass
