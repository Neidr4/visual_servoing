#! /usr/bin/env python

import rospy
import cv2
import numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

focal_length_px = 580

class VS:
    def __init__ (self, focal_length_px):
        # Subscriber
        self.sub_cam = rospy.Subscriber('/rms/camera1/image_raw', 
            Image, self.callback_cam)
        # Variables
        self.bridge = CvBridge()
        self.cv_image = numpy.ndarray((480, 640, 4)) # Image()
        self.last_image = Image()
        self.triple_jacobian = numpy.empty((0, 6), float)
        self.focal_length_px = focal_length_px

    def callback_cam(self, image_raw):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
            # self.display_image("bonsoir", self.cv_image)
        except CvBridgeError as e:
            print(e)

    def compute_ij(self, list_of_point):
        self.clear_jacobian()
        for point in list_of_point:
            if isinstance (point, Point):
                jacobian_matrix = numpy.matrix([
                    [-self.focal_length_px/point.Z, 0, point.u/point.Z, (point.u*point.v)/self.focal_length_px, -(self.focal_length_px+numpy.sqrt(point.u)/self.focal_length_px), point.v],
                    [0, -self.focal_length_px/point.Z, point.v/point.Z, self.focal_length_px+numpy.sqrt(point.v)/self.focal_length_px, -(point.u*point.v)/self.focal_length_px, -point.u]])

                self.triple_jacobian = numpy.append(self.triple_jacobian, 
                    jacobian_matrix, axis=0)
            else:
                rospy.loginfo("The argment passed in compute_ij, is not Point")
        print("robot_0.triple_jacobian size is: " + str(self.triple_jacobian.shape))

    def clear_jacobian(self):
        self.triple_jacobian = numpy.empty((0, 6), float)

    def display_image(self, name, img):
        cv2.imshow(name, img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def shutdown_function(self):
        # cv2.waitKey(0)
        cv2.destroyAllWindows()


class Point(object):
    """docstring for Point"""
    def __init__(self, name, u, v, Z):
        super(Point, self).__init__()
        self.name = name
        self.u = u
        self.v = v
        self.Z = Z

    def print_coordinates(self):
        print("Coordinates of "+str(self.name)+" is (" 
            +str(self.u)+", "+str(self.v)+", "+str(self.Z)+")")

    def add_circle(self, img):
        # Center coordinates
        center_coordinates_check = [self.u, self.v]

        if(self.u > img.shape[1]):
            center_coordinates_check[0] = img.shape[1]
        else:
            center_coordinates_check[0] = self.u
        
        if(self.u > img.shape[0]):
            center_coordinates_check[1] = img.shape[0]
        else:
            center_coordinates_check[1] = self.v

        center_coordinates = tuple(center_coordinates_check)

        # Radius of circle
        radius = 20
        # Blue color in BGR
        color = (255, 0, 0)
        # Line thickness of 2 px
        thickness = 2
          
        # Using cv2.circle() method
        # Draw a circle with blue line borders of thickness of 2 px
        img_modified = cv2.circle(img, center_coordinates, radius, color, thickness)
        return img_modified

def get_param():
    try:
        global global_topic_name
        global_topic_name = rospy.get_param("/visual_servoing_node/video_feed_topic")
        pass
    except rospy.KeyRaise as e:
        raise e
    

def shutdown_function():
    rospy.loginfo("Thanks for using Visual Servoing - Shuting Down")
    robot_0.shutdown_function()


if __name__ == '__main__':
    rospy.loginfo("Initlialisation of node visual_servoing_node")
    rospy.init_node("visual_servoing_node")
    rate = rospy.Rate(10)

    rospy.loginfo("Fetching parameters")
    get_param()
    print("global_topic_name is:" + str(global_topic_name))


    rospy.loginfo("Creation of instance of VS")
    robot_0 = VS(focal_length_px=580)
    rospy.on_shutdown(shutdown_function)

    rospy.loginfo("Creation of three instance of Point")
    point_1 = Point("Point_1", 200, 200, 200)
    point_2 = Point("Point_2", 400, 200, 200)
    point_3 = Point("Point_3", 300, 400, 200)
    list_of_point = [point_1, point_2, point_3]


    rospy.loginfo("Entering loop")
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Start of Cycle")

            # if rospy.has_param('/visual_servoing_node/video_feed_topic'):
                
            #     print("global_topic_name is:" + str(global_topic_name))
            # else:
            #     print("did not found the parameter")

            point_1.add_circle(robot_0.cv_image)
            point_2.add_circle(robot_0.cv_image)
            point_3.add_circle(robot_0.cv_image)
            robot_0.display_image("robot_0", robot_0.cv_image)

            robot_0.compute_ij(list_of_point)

            rate.sleep()
            rospy.loginfo("End of Cycle")
            # rospy.spin()
        except rospy.ROSInterruptException:
            pass