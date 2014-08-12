import roslib
roslib.load_manifest('RavenDebridement')


import IPython
import cv
import cv2
import cv_bridge
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray

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

        self.left_points = None
        self.right_points = None

        #========SUBSCRIBERS========#
        # image subscribers
        rospy.Subscriber("/BC/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/BC/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/BC/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/BC/right/camera_info",
                         CameraInfo, self.right_info_callback)

        #========PUBLISHERS=========#
        self.black_dot_segmented_pub = rospy.Publisher("/gak/black_dot_segmented", Image)
        self.black_dot_point_pub = rospy.Publisher("/gak/black_dot_point", PointStamped)
        self.grasp_point_pub = rospy.Publisher("/gak/grasp_point", PoseStamped)
        self.grasp_poses_pub = rospy.Publisher("/gak/grasp_poses", PoseArray)



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
        self.right_points = self.process_image(self.right_image)

    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.left_points = self.process_image(self.left_image)
        
        poses = []
        if self.right_points is not None and self.left_points is not None:
            self.left_points.sort(key=lambda x:x[0])
            self.right_points.sort(key=lambda x:x[0])
            disparities = self.assign_disparities(self.left_points, self.right_points)
            for i in range(len(self.left_points)):
                x = self.left_points[i][0]
                y = self.left_points[i][1]
                disparity = disparities[i]
                print x,y,disparity
                pt = Util.convertStereo(x,y,disparity, self.info)
                pt = tfx.point(pt)
                pt = tfx.convertToFrame(pt, '/two_remote_center_link')
                pose = tfx.pose(pt)
                pose = pose.as_tf()*tfx.pose(tfx.tb_angles(-90,0,180)).as_tf()*tfx.pose(tfx.tb_angles(90,0,0))
                poses.append(pose)

        print poses

        pose_array = PoseArray()
        pose_array.header = poses[0].msg.PoseStamped().header
        for pose in poses:
            pose_array.poses.append(pose)
        self.grasp_poses_pub.publish(pose_array)

        # print self.left_point, self.right_point
        # if self.right_point is not None and self.left_point is not None:
        #     disparity = abs(self.right_point[0]-self.left_point[0])
        #     pt = Util.convertStereo(self.left_point[0], self.left_point[1], disparity, self.info)
        #     pt = tfx.point(pt)
        #     print pt
        #     self.black_dot_point_pub.publish(pt.msg.PointStamped())

        # pt = tfx.convertToFrame(pt, '/two_remote_center_link')
        # pose = tfx.pose(pt)
        # pose = pose.as_tf()*tfx.pose(tfx.tb_angles(-90,0,180)).as_tf()*tfx.pose(tfx.tb_angles(90,0,0))
        # # pose = pose.as_tf()*tfx.pose(tfx.tb_angles(180,0,0)).as_tf()*tfx.pose(tfx.tb_angles(0,-75,0))
        # self.grasp_point_pub.publish(pose.msg.PoseStamped())        

    def assign_disparities(self, left_centroids, right_centroids):
        window_height = 10
        # make copies to sort
        # left_centroids = left_centroids.copy()
        # right_centroids = right_centroids.
        
        left_disparities = [None for i in range(len(left_centroids))]
        right_disparities = [None for i in range(len(left_centroids))]

        # assign the disparities
        for a in range(len(left_centroids)):
            for b in range(len(right_centroids)):
                # pixel y locations
                p1 = left_centroids[a][1]
                p2 = right_centroids[b][1]
                if p1-window_height<p2 and p1+window_height>p2 and right_disparities[b] is None:
                    disparity = abs(left_centroids[a][0]-right_centroids[b][0])
                    left_disparities[a] = disparity
                    right_disparities[b] = disparity
                    break
        return left_disparities



    def image_publisher(self, image, image_publisher):
        img_msg = self.bridge.cv_to_imgmsg(cv.fromarray(image))
        image_publisher.publish(img_msg)

    def process_image(self, image):
        # image = cv2.cvtColor(image,  cv2.COLOR_BGR2GRAY)
        # ret, image = cv2.threshold(image, 60, 255, cv2.THRESH_BINARY_INV)
        # # make a window
        # image[:400] = 0
        # image[:,:100] = 0
        # image[:,800:] = 0
        # erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # image = cv2.erode(image, erosion_kernel)
        # image = cv2.erode(image, erosion_kernel)
        # image = cv2.erode(image, erosion_kernel)
        # image = cv2.dilate(image, erosion_kernel)

        # contours, contour_hierarchy = cv2.findContours(image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        # # print len(contours)
        # if len(contours) != 1:
        #     return None


        # moments = cv2.moments(contours[0])
        # if moments['m00'] != 0.0:
        #     cx = moments['m10']/moments['m00']
        #     cy = moments['m01']/moments['m00']
        #     centroid = (cx,cy)
        #     return centroid

        original_image = image.copy()  # save a copy of the original

        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image = cv2.inRange(image, np.array([70,40,100]),np.array([80,255,255]))
        erosion_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image = cv2.erode(image, erosion_kernel)

        # create the contours
        contours, contour_hierarchy = cv2.findContours(image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # remove contours that are too small
        filtered = []
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 5000:
                filtered.append(contours[i])
        contours = filtered
        # get a convex hull
        for i in range(len(contours)):
            contours[i] = cv2.convexHull(contours[i])

        # simplify the contours
        for i in range(len(contours)):
            contours[i] = cv2.approxPolyDP(contours[i],2,True)

        # get the largest contour
        image[:] = 0
        cv2.drawContours(image, [contours[0]], -1, 255, -1)

        mask = image

        image = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
        thresholded_image1 = cv2.inRange(image, np.array([160,70,0]),np.array([180,255,80]))
        thresholded_image2 = cv2.inRange(image, np.array([0,70,0]),np.array([20,255,80]))
        image = thresholded_image1 + thresholded_image2

        # create the contours
        contours, contour_hierarchy = cv2.findContours(image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # remove contours that are too small
        filtered = []
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 100:
                filtered.append(contours[i])
        contours = filtered
        # # get a convex hull
        # for i in range(len(contours)):
        #     contours[i] = cv2.convexHull(contours[i])

        cv2.drawContours(image, contours, -1, 255, -1)

        image = cv2.bitwise_and(image, mask)

        dilation_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        image = cv2.dilate(image, dilation_kernel)
        image = cv2.dilate(image, dilation_kernel)

        if DEBUG:
            self.image_publisher(image, self.black_dot_segmented_pub)

        # get the contours in the masked image
        contours, contour_hierarchy = cv2.findContours(image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        

        centroids = []
        for contour in contours:
            moments = cv2.moments(contour)
            if moments['m00'] != 0.0:
                cx = moments['m10']/moments['m00']
                cy = moments['m01']/moments['m00']
                centroid = (cx,cy)
                centroids.append(centroid)
        return centroids


if __name__ == "__main__":
    rospy.init_node('gak_black_dot_estimator')
    a = BlackDotEstimator()
    rospy.spin()