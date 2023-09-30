#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthCameraNode:
    def __init__(self):
        rospy.init_node('depth_camera_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.image_callback)
        self.frame_count = 0
        self.save_frame = False

    def image_callback(self, msg):
        if self.frame_count == 9:
            self.save_frame = True
        if self.save_frame:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # Save the image to a txt file
                with open('depth_image.txt', 'w') as txt_file:
                    for row in cv_image:
                        txt_file.write(' '.join(map(str, row)) + '\n')
                rospy.loginfo('Saved the 10th frame to depth_image.txt')
                self.save_frame = False
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

