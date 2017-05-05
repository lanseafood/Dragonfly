import rospy
from std_msgs.msg import Int64
import XY_position.msg

def talker():
    pub = rospy.Publisher('chatter', int64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pose = [1, -1, 0, 0, 2, -2]

    while not rospy.is_shutdown():
        poses = 1
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass