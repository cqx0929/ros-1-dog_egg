#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
from std_msgs.msg import String
class DepthCameraNode:
    def __init__(self):
        rospy.init_node('depth_camera_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback)
        self.frame_count = 0
        self.save_frame = False
        self.image_pub = rospy.Publisher('image_msg', String, queue_size = 1)

    def image_callback(self, msg):
        if self.frame_count == 9:
            self.save_frame = True
        if self.save_frame:
            try:
                depth_image_msg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                image_pub.publish(depth_image_msg)
		rospy.loginfo('publishing')
            except CvBridgeError as e:
                rospy.logerr(e)

        self.frame_count += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        depth_camera_node = DepthCameraNode()
        depth_camera_node.run()
    except rospy.ROSInterruptException:
        pass

