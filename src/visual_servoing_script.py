#! /usr/bin/env python

import rospy
import cv2
import numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

focal_length_px = 580

class VS:
    def __init__ (self, name, video_topic, focal_length_px):
        # Variables
        self.name = name
        self.init_camera = True
        self.bridge = CvBridge()
        self.cv_image = numpy.ndarray((480, 640, 4)) # Image()
        self.mask = numpy.ndarray((480, 640, 4)) # Image()
        self.image_with_circle = numpy.ndarray((480, 640, 4)) # Image()
        self.last_image = Image()
        self.image_size = numpy.array([0, 0])
        self.triple_jacobian = numpy.empty((0, 6), float)
        self.focal_length_px = focal_length_px

        #Colors
        self.green_lower = numpy.array([40,30,30])
        self.green_upper = numpy.array([70,255,255])
        self.blue_lower = numpy.array([80, 30, 30])
        self.blue_upper = numpy.array([120,255,255])
        self.yellow_lower = numpy.array([30, 80, 100])
        self.yellow_upper = numpy.array([60, 255, 255])

        # Subscriber
        self.sub_cam = rospy.Subscriber(video_topic, 
            Image, self.callback_cam)
        image_pub_string = "/vs/" + str(self.name) + "/image_pub"
        self.image_pub = rospy.Publisher(image_pub_string, Image)

    def callback_cam(self, image_raw):
        try:
            # Converting from ROS image to OpenCV
            self.cv_image = self.bridge.imgmsg_to_cv2(image_raw, "bgr8")
            if self.init_camera == True:
                # Fetching the shape of the cv_image
                self.image_size[0] = self.cv_image.shape[0]
                self.image_size[1] = self.cv_image.shape[1]
                self.init_camera = False
                rospy.loginfo("Init_camera image_size = " + str(self.image_size))
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
        #print("robot_0.triple_jacobian size is: " + str(self.triple_jacobian.shape))

    def get_features_pos(self):
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        #Detection of color in hsv image
        self.mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        #Image display
        #cv2.imshow("cv2_image", cv2_image)
        #cv2.imshow("hsv", hsv)

        point_1.set_uv_to_0()
        point_2.set_uv_to_0()
        point_3.set_uv_to_0()

        mask_rows = self.mask.shape[0]
        mask_columns = self.mask.shape[1]

        for i in range(mask_rows):
            for j in range(mask_columns):
                # Checking if the color of the pixel is not black
                if(self.mask[i, j] != 0):
                    # Checking for first point
                    if(point_1.u == 0 and point_1.v == 0):
                        # Assigning first point pos to point_1
                        point_1.u = j
                        point_1.v = i
                    # Assigning each non-black point to point_3
                    # The remaining point will be the last
                    point_3.u = j
                    point_3.v = i

        # Computing the middle point u and v
        point_2.u = (point_1.u + point_3.u )/2
        point_2.v = (point_1.v + point_3.v )/2

      #  point_1.print_coordinates()
      #  point_2.print_coordinates()
      #  point_3.print_coordinates()

      #  self.mask = point_1.add_circle(self.mask)
      #  self.mask = point_3.add_circle(self.mask)
      #  cv2.imshow("get_features_pos", self.mask)
      #  cv2.waitKey(0)
      #  cv2.destroyAllWindows()


    def clear_jacobian(self):
        self.triple_jacobian = numpy.empty((0, 6), float)

    def display_image(self, name, img):
        cv2.imshow(name, img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
      #  if cv2.waitKey(33) == ord('q'):
      #      cv2.destroyAllWindows()
      #  else:
      #      print("press \"q\" for closing the window")

    def publish_image(self, cv_image):
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


    def shutdown_function(self):
        # cv2.waitKey(0)
        cv2.destroyAllWindows()


class Point(object):
    """docstring for Point"""
    def __init__(self, name, u, v, Z, color):
        super(Point, self).__init__()
        self.name = name
        self.u = u
        self.v = v
        self.Z = Z
        self.color = color

    def print_coordinates(self):
        print("Coordinates of "+str(self.name)+" is (" 
            +str(self.u)+", "+str(self.v)+", "+str(self.Z)+")")

    def set_uv_to_0(self):
        self.u = 0
        self.v = 0

    def add_circle(self, img):
        # Center coordinates
        center_coordinates_check = [self.u, self.v]

      #  if(self.u > img.shape[1]):
      #      center_coordinates_check[0] = img.shape[1]
      #  else:
      #      center_coordinates_check[0] = self.u
      #  
      #  if(self.u > img.shape[0]):
      #      center_coordinates_check[1] = img.shape[0]
      #  else:
      #      center_coordinates_check[1] = self.v

        center_coordinates = tuple(center_coordinates_check)

        # Radius of circle
        radius = 10
        # Blue color in BGR
        color = self.color
        # Line thickness of 2 px
        thickness = 2
          
        # Using cv2.circle() method
        # Draw a circle with blue line borders of thickness of 2 px
        img_modified = cv2.circle(img, center_coordinates, radius, color, thickness)
        return img_modified

    def distance_from(self, other_point):
        if isinstance (other_point, Point):
            u = self.u - other_point.u
            v = self.v - other_point.v
            distance_tuple = (u, -v)
            print("distance between the two points: " + str(distance_tuple))
            return distance_tuple
        else:
            print("Parameters passed is not a point")
            return 0


def get_param():
    global video_feed_topic
    video_feed_topic = rospy.get_param("/visual_servoing_node/video_feed_topic")
    global focal_length_pxl
    focal_length_pxl = rospy.get_param("/visual_servoing_node/focal_length_pxl")
    global desired_point_1_x
    desired_point_1_x = rospy.get_param("/visual_servoing_node/desired_point_1_x")
    global desired_point_1_y
    desired_point_1_y = rospy.get_param("/visual_servoing_node/desired_point_1_y")
    global desired_point_2_x
    desired_point_2_x = rospy.get_param("/visual_servoing_node/desired_point_2_x")
    global desired_point_2_y
    desired_point_2_y = rospy.get_param("/visual_servoing_node/desired_point_2_y")
    global desired_point_3_x
    desired_point_3_x = rospy.get_param("/visual_servoing_node/desired_point_3_x")
    global desired_point_3_y
    desired_point_3_y = rospy.get_param("/visual_servoing_node/desired_point_3_y")
    

def shutdown_function():
    rospy.loginfo("Thanks for using Visual Servoing - Shuting Down")
    robot_0.shutdown_function()


if __name__ == '__main__':
    rospy.loginfo("Initlialisation of node visual_servoing_node")
    rospy.init_node("visual_servoing_node")
    rate = rospy.Rate(10)

    rospy.loginfo("Fetching parameters")
    get_param()

    rospy.loginfo("Creation of instance of VS")
    robot_0 = VS(name="robot_0",
        video_topic=video_feed_topic, 
        focal_length_px=focal_length_pxl)
    rospy.on_shutdown(shutdown_function)

    rospy.loginfo("Creation of three instance of Point")

   # desired_point_1 = Point("Desired_Point_1", 360, 200, 200, (255, 0, 255))
   # desired_point_2 = Point("Desired_Point_2", 360, 300, 200, (0, 255, 255))
   # desired_point_3 = Point("Desired_Point_3", 360, 400, 200, (255, 255, 0))
    desired_point_1 = Point("Desired_Point_1", desired_point_1_x, desired_point_1_y, 200, (255, 0, 255))
    desired_point_2 = Point("Desired_Point_2", desired_point_2_x, desired_point_2_y, 200, (0, 255, 255))
    desired_point_3 = Point("Desired_Point_3", desired_point_3_x, desired_point_3_y, 200, (255, 255, 0))
    point_1 = Point("Point_1", 200, 200, 200, (0, 0, 255))
    point_2 = Point("Point_2", 400, 200, 200, (0, 255, 0))
    point_3 = Point("Point_3", 300, 400, 200, (255, 0, 0))
    list_of_point = [point_1, point_2, point_3]
    first_try = 0


    rospy.loginfo("Entering loop")
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Start of Cycle")

            if first_try == 0:
                first_try = 1
            else:
                robot_0.get_features_pos()
                robot_0.publish_image(robot_0.image_with_circle)

            robot_0.image_with_circle = desired_point_1.add_circle(robot_0.cv_image)
            robot_0.image_with_circle = desired_point_2.add_circle(robot_0.cv_image)
            robot_0.image_with_circle = desired_point_3.add_circle(robot_0.cv_image)
          #  desired_point_1.print_coordinates()
          #  desired_point_2.print_coordinates()
          #  desired_point_3.print_coordinates()
            robot_0.image_with_circle = point_1.add_circle(robot_0.cv_image)
            robot_0.image_with_circle = point_2.add_circle(robot_0.cv_image)
            robot_0.image_with_circle = point_3.add_circle(robot_0.cv_image)

            robot_0.mask = point_1.add_circle(robot_0.cv_image)
            robot_0.mask = point_2.add_circle(robot_0.cv_image)
            robot_0.mask = point_3.add_circle(robot_0.cv_image)

            robot_0.display_image("robot_0_mask", robot_0.mask)

            robot_0.compute_ij(list_of_point)
            print("desired_point_1.distance_from(point_1): " + 
                    str(desired_point_1.distance_from(point_1)))
            print("desired_point_2.distance_from(point_2): " + 
                    str(desired_point_2.distance_from(point_2)))
            print("desired_point_3.distance_from(point_3): " + 
                    str(desired_point_3.distance_from(point_3)))

            rate.sleep()
            rospy.loginfo("End of Cycle")
            # rospy.spin()
        except rospy.ROSInterruptException:
            shutdown_function()
            pass
