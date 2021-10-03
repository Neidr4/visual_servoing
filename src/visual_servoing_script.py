#! /usr/bin/env python

import rospy
import cv2
import numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# global_name = rospy.get_param("/vs/video_feed_topic")

class VS:
    def __init__ (self):
        # Subscriber
        self.sub_cam = rospy.Subscriber('/rms/camera1/image_raw', 
            Image, self.callback_cam)
        # Variables
        self.bridge = CvBridge()
        self.cv_image = numpy.ndarray((480, 640, 4)) # Image()
        self.last_image = Image()
        self.triple_jacobian = numpy.empty((0, 6), float)

    def callback_cam(self, image_raw):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
            # self.display_image("bonsoir", self.cv_image)
        except CvBridgeError as e:
            print(e)

    def display_image(self, name, img):
        cv2.imshow(name, img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def shutdown_function(self):
        # cv2.waitKey(0)
        cv2.destroyAllWindows()


def shutdown_function():
    rospy.loginfo("Thanks for using Visual Servoing - Shuting Down")
    robot_0.shutdown_function()


if __name__ == '__main__':
    rospy.loginfo("Initlialisation of node visual_servoing_node")
    rospy.init_node("visual_servoing_node")
    rate = rospy.Rate(10)

    rospy.loginfo("Creation of instance of VS")
    robot_0 = VS()
    rospy.on_shutdown(shutdown_function)

    

    rospy.loginfo("Entering loop")
    while not rospy.is_shutdown():
        try:
            rate.sleep()
            # rospy.spin()
        except rospy.ROSInterruptException:
            pass