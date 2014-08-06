import roslib
roslib.load_manifest('RavenDebridement')


import IPython
import cv
import cv2
import cv_bridge
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped

import tfx
import RavenDebridement.foam_util as Util

DEBUG = True


class BlackDotEstimator(object):
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.left_image = None
        self.right_image = None
        # TODO: are all of them necessary?
        self.info = {'l': None, 'r': None, 'b': None, 'd': None}

        self.left_point = None
        self.right_point = None

        #========SUBSCRIBERS========#
        # image subscribers
        rospy.Subscriber("/BC/left/image_rect", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/BC/right/image_rect", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/BC/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/BC/right/camera_info",
                         CameraInfo, self.right_info_callback)

        #========PUBLISHERS=========#
        self.black_dot_segmented_pub = rospy.Publisher("/gak/black_dot_segmented", Image)
        self.black_dot_point_pub = rospy.Publisher("/gak/black_dot_point", PointStamped)


    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.right_point = self.process_image(self.right_image)

    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.left_point = self.process_image(self.left_image)
        print self.left_point, self.right_point
        if self.right_point is not None and self.left_point is not None:
            disparity = abs(self.right_point[0]-self.left_point[0])
            print "hello"
            pt = Util.convertStereo(self.left_point[0], self.left_point[1], disparity, self.info)
            pt = tfx.point(pt)
            print pt
            self.black_dot_point_pub.publish(pt.msg.PointStamped())



    def image_publisher(self, image, image_publisher):
        img_msg = self.bridge.cv_to_imgmsg(cv.fromarray(image))
        image_publisher.publish(img_msg)

    def process_image(self, image):
        image = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
        ret, image = cv2.threshold(image, 40, 255, cv2.THRESH_BINARY_INV)
        # make a window
        image[:300] = 0
        image[:,:100] = 0
        image[:,900:] = 0
        erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image = cv2.erode(image, erosion_kernel)
        image = cv2.erode(image, erosion_kernel)

        if DEBUG:
            self.image_publisher(image, self.black_dot_segmented_pub)

        contours, contour_hierarchy = cv2.findContours(image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # print len(contours)
        if len(contours) != 1:
            return None


        moments = cv2.moments(contours[0])
        if moments['m00'] != 0.0:
            cx = moments['m10']/moments['m00']
            cy = moments['m01']/moments['m00']
            centroid = (cx,cy)
            return centroid


if __name__ == "__main__":
    rospy.init_node('gak_black_dot_estimator')
    a = BlackDotEstimator()
    rospy.spin()