#! /usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from math import *

class PosePublisher:
    def __init__(self, in_topic_name: str, out_topic_name: str) -> None:
        self.data_to_publish = PoseStamped()
        self.pub = rospy.Publisher(out_topic_name, PoseStamped, queue_size=10)
        rospy.Subscriber(in_topic_name, TransformStamped, self.callback)
        rospy.spin()

    def callback(self, data):
        self.data_to_publish.header = data.header #take main heading with time stamp
        #data_to_publish.pose.position = data.transform.translation
        self.data_to_publish.pose.orientation = data.transform.rotation

        # transformations to get ENU frame coordinate
        self.data_to_publish.pose.position.x = - data.transform.translation.y
        self.data_to_publish.pose.position.y = data.transform.translation.x
        self.data_to_publish.pose.position.z = data.transform.translation.z
            
            
        roll,pitch,yaw = tf.transformations.euler_from_quaternion((data.transform.rotation.x,
                                                                    data.transform.rotation.y,
                                                                    data.transform.rotation.z,
                                                                    data.transform.rotation.w))
            
        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw+radians(90))
        self.data_to_publish.pose.orientation.x = q[0]
        self.data_to_publish.pose.orientation.y = q[1]
        self.data_to_publish.pose.orientation.z = q[2]
        self.data_to_publish.pose.orientation.w = q[3]
            

        self.pub.publish(self.data_to_publish)
        # print(data_to_publish)

if __name__ == '__main__':
    try:
        rospy.init_node('convert2ros', anonymous=True)
        in_topic = rospy.get_param('~in_topic')
        out_topic = rospy.get_param('~out_topic')
        PosePublisher(in_topic, out_topic)
    except rospy.ROSInterruptException:
        pass
