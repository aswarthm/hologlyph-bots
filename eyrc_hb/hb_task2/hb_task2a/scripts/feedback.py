#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import sensor_msgs.msg
import math
import cv2
from cv_bridge import CvBridge
import numpy as np

# Import the required modules
##############################################################


class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.get_logger().info("feedback start")

        self.cv_bridge = CvBridge()

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(arucoDict, parameters)

        # self.DetectedArucoMarkers = {}
        # self.ArUco_details_dict = {}
        # self.ArUco_corners = {}
        # self.ArUco_marker_angles = {}

        # Subscribe the topic /camera/image_raw
        self.subscription = self.create_subscription(sensor_msgs.msg.Image,
                                                     "/camera/image_raw",
                                                     self.image_callback,
                                                     10)
        
        self.publisher = self.create_publisher(Pose2D,
                                               "/detectedAruco",
                                               10)

    def image_callback(self, msg):
        # convert ROS image to opencv image
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.get_logger().info(str(type(cv_image)))
        DetectedArucoMarkers = {}
        ArucoDetailsDict = {}
        ArucoCorners = {}
        ArucoMarkerAngles = {}
        corners, ids, rejected = self.arucoDetector.detectMarkers(img)
        for i in range(0, ids.shape[0]):
            DetectedArucoMarkers[int(ids[i][0,])] = corners[i][0,]

        ArucoMarkerAngles = self.getOrientationDeg(DetectedArucoMarkers)
        # ArucoMarkerAngles = self.Calculate_orientation_in_degree(DetectedArucoMarkers)

        for i in range(0, ids.shape[0]):
            center = np.mean(corners[i][0], axis=0)
            ArucoDetailsDict[int(ids[i][0,])] = [
                center, ArucoMarkerAngles[int(ids[i][0,])]]
            ArucoCorners[int(ids[i][0,])] = corners[i][0,]

        img = self.mark_ArUco_image(img, ArucoDetailsDict, ArucoCorners)

        # self.get_logger().info(str(rejected))
        # img = self.mark_ArUco_image(img, self.DetectedArucoMarkers, self.DetectedArucoMarkers)
        cv2.imshow("lol", img)
        cv2.waitKey(1)

        self.get_logger().info(str(ArucoDetailsDict[int(ids[0][0])]))#################################################verify if bot coords is same as gazebo coords

        botTwist = Pose2D()
        botTwist.x = 0.0
        botTwist.y = 0.0
        botTwist.theta = 0.0

        self.publisher.publish(botTwist)

        # Detect Aruco marker
        # Publish the bot coordinates to the topic  /detected_aruco

    def getOrientationDeg(self, DetectedArucoMarkers):
        ArucoMarkerAngles = {}
        cnt = 0
        ans_x = 0
        ans_y = 0
        midpoint_x = 0
        midpoint_y = 0
        right_x = 0
        right_y = 0
        for i in DetectedArucoMarkers:
            for j in DetectedArucoMarkers[i]:
                if (cnt % 4) == 0:
                    ans_x = j[0,]
                    ans_y = j[1,]
                    midpoint_x = ans_x
                    midpoint_y = ans_y
                    cnt = 1
                else:
                    cnt += 1
                    if cnt == 2:
                        midpoint_x = (midpoint_x + j[0,])/2
                        midpoint_y = (midpoint_y + j[1,])/2
                        right_x = j[0,]
                        right_y = j[1,]
                    ans_x += j[0,]
                    ans_y += j[1,]
            ans_x = int(ans_x/4)
            ans_y = int(ans_y/4)
            midpoint_x = int(midpoint_x)
            midpoint_y = int(midpoint_y)
            midpoint_x = midpoint_x - ans_x
            midpoint_y = -(midpoint_y - ans_y)

            ans_x = 0
            ans_y = 0

            angle = 0.0

            if midpoint_y < 0:
                angle = 360.0-np.arccos(np.inner([1, 0], [midpoint_x, midpoint_y])/np.linalg.norm([midpoint_x, midpoint_y]))*180.0/np.pi
            else:
                angle = np.arccos(np.inner([1, 0], [midpoint_x, midpoint_y])/np.linalg.norm([midpoint_x, midpoint_y]))*180.0/np.pi

            angle = round(angle, 2)
            self.get_logger().info(str(angle))
            ArucoMarkerAngles[i] = angle

        # returning the angles of the ArUco markers in degrees as a dictionary
        return ArucoMarkerAngles

    def mark_ArUco_image(self, image, ArucoDetailsDict, ArucoCorners):

        for ids, details in ArucoDetailsDict.items():
            center = details[0]
            center = [int(center[0]), int(center[1])]
            thickn = 1
            cv2.circle(image, center, thickn, (0, 0, 255), -1)

            corner = ArucoCorners[int(ids)]
            cv2.circle(image, (int(corner[0][0]), int(
                corner[0][1])), thickn, (50, 50, 255), -1)
            # cv2.circle(image, (int(corner[1][0]), int(corner[1][1])), thickn, (0, 255, 0), -1)
            # cv2.circle(image, (int(corner[2][0]), int(corner[2][1])), thickn, (128, 0, 255), -1)
            # cv2.circle(image, (int(corner[3][0]), int(corner[3][1])), thickn, (25, 255, 255), -1)

            tl_tr_center_x = int((corner[0][0] + corner[1][0]) / 2)
            tl_tr_center_y = int((corner[0][1] + corner[1][1]) / 2)

            cv2.line(image, center, (tl_tr_center_x,
                     tl_tr_center_y), (255, 0, 0), thickn)
            display_offset = int(
                math.sqrt((tl_tr_center_x - center[0])**2+(tl_tr_center_y - center[1])**2))
            cv2.putText(image, str(ids), (center[0]+int(display_offset/2)+5,
                        center[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), thickn)
            angle = details[1]
            cv2.putText(image, str(
                angle), (center[0]-display_offset, center[1]+10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
        return image


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
