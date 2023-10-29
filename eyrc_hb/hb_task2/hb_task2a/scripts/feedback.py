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
*****************************************************************************************
'''


# Team ID:		    hb_1036
# Author List:		[ M Aswartha Reddy, D K Bharath Reddy, Pulkit Dhamija, Sangeeta Prasad ]
# Filename:		    feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		
#                   Subs: [ /camera/image_raw ]
#                   Pubs: [ /detectedAruco ]


################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import sensor_msgs.msg
import math
import cv2
from cv_bridge import CvBridge
import numpy as np

from nav_msgs.msg import Odometry
from math import sin, cos
from tf_transformations import euler_from_quaternion

# Import the required modules
##############################################################


class ArUcoDetector(Node):

    def __init__(self):
        '''
            Purpose:
            ---
            Called when the ArucoDetector Node is initialised

            Input Arguments:
            ---
            self:ArUcoDetector

            Returns:
            ---
            None

            Example call:
            ---
            -
        '''
        super().__init__('ar_uco_detector')
        self.get_logger().info("feedback start")

        self.cv_bridge = CvBridge()

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(arucoDict, parameters)

        # Subscribe the topic /camera/image_raw
        self.subscription = self.create_subscription(sensor_msgs.msg.Image,
                                                     "/camera/image_raw",
                                                     self.image_callback,
                                                     10)
        
        self.publisher = self.create_publisher(Pose2D,
                                               "/detectedAruco",
                                               10)

        self.bot_path = []

    def image_callback(self, msg):
        '''
        Purpose:
        ---
        Callback function when new image is published by Gazebo. Calculates postion of markers on the arena, and publishes bot's location

        Input Arguments:
        ---
        self:ArUcoDetector
        msg:sensor_msgs.msg.Image Image as viewed by Gazebo Camera

        Returns:
        ---
        None

        Example call:
        ---
        -
        '''
        # convert ROS image to opencv image
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen_kernel = np.array([                                         #this works better
                                    [0,-1,0], 
                                    [-1,6,-1], 
                                    [0,-1,0]
                                  ])
        img_sharp = cv2.filter2D(img, -1, sharpen_kernel)

        # Detect Aruco marker
        DetectedArucoMarkers = {}
        ArucoDetailsDict = {}
        ArucoCorners = {}
        ArucoMarkerAngles = {}
        corners, ids, rejected = self.arucoDetector.detectMarkers(img_sharp)
        '''
            id 1    bot
            id 8    top left
            id 10   top right
            id 12   bott right
        '''
        for i in range(0, ids.shape[0]):
            DetectedArucoMarkers[int(ids[i][0,])] = corners[i][0,]

        ArucoMarkerAngles = self.getOrientationDeg(DetectedArucoMarkers)

        for i in range(0, ids.shape[0]):
            center = np.mean(corners[i][0], axis=0)
            ArucoDetailsDict[int(ids[i][0,])] = [
                center, ArucoMarkerAngles[int(ids[i][0,])]]
            ArucoCorners[int(ids[i][0,])] = corners[i][0,]

        # img = cv2.aruco.drawDetectedMarkers(image = img, corners = corners, ids=ids, borderColor=(0, 255, 0))
        img = self.mark_ArUco_image(img, ArucoDetailsDict, ArucoCorners)

        # self.get_logger().info(str(rejected))
        # img = self.mark_ArUco_image(img, self.DetectedArucoMarkers, self.DetectedArucoMarkers)

        # self.get_logger().info(str(ids))

        # Publish the bot coordinates to the topic  /detected_aruco
        try:
            bot_loc = ArucoDetailsDict[1] #get bot id's details
            # msg = "aru" + str(round(bot_loc[0][0], 2)) + "\t" + str(round(bot_loc[0][1], 2)) + "\t" + str(round(bot_loc[1], 2))
            # self.get_logger().info(msg)#################################################verify if bot coords is same as gazebo coords
            # msg =  "cur" + str(round(self.hb_x, 2)) + "\t" + str(round(self.hb_y, 2)) + "\t" + str(round(self.hb_theta*180.0/math.pi, 2))
            # self.get_logger().info(msg)

            arenaCenter = self.calibrateCenter(ArucoDetailsDict)

            #skipping theta calibration, by assuming it doesnt matter, calibrate if required
            botCenterX = bot_loc[0][0] - arenaCenter[0]
            botCenterY = bot_loc[0][1] - arenaCenter[1]
            botTheta = bot_loc[1] + 4.24  ##############debug change this value later, do not hardcode values. 4.24 because aruco marker is not exactly 0degress to baase of bot

            self.bot_path.append((int(bot_loc[0][0]), int(bot_loc[0][1])))
            # self.get_logger().info(str(self.bot_path))

            # msg =  "cal" + str(round(botCenterX, 2)) + " " + str(round(botCenterY, 2)) + " " + str(round(bot_loc[1], 2))
            self.publishBotLocation([botCenterX, botCenterY, botTheta]) #[x, y, theta]

        except KeyError as e:
            #bot id not found or corner ids not found
            #either publish panic message to stop bot or just pass, assume id will be detected in next loop
            #better to stop bot as it will prevent it from rolling out of arena
            # self.get_logger().error(str(e.message))
            self.get_logger().error("Some Aruco Tags Were Not Detected")
        except Exception as e:
            self.get_logger().error(str(e))

        cv2.imshow("lol", img)
        cv2.waitKey(1)

    def getOrientationDeg(self, DetectedArucoMarkers):
        '''
        Purpose:
        ---
        Calculates the angle of aruco marker based on the slopes of sides. Angle is wrt y axis

        Input Arguments:
        ---
        self:ArUcoDetector
        DetectedArucoMarkers:List

        Returns:
        ---
        ArucoMarkerAngles:List Contains the angles of all the detected aruco markers in degrees

        Example call:
        ---
        ArucoMarkerAngles = self.getOrientationDeg(DetectedArucoMarkers)
        '''
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
                        midpoint_x = (midpoint_x + j[0,])/2.0
                        midpoint_y = (midpoint_y + j[1,])/2.0
                        right_x = j[0,]
                        right_y = j[1,]
                    ans_x += j[0,]
                    ans_y += j[1,]
            ans_x = ans_x/4.0
            ans_y = ans_y/4.0
            midpoint_x = midpoint_x
            midpoint_y = midpoint_y
            midpoint_x = midpoint_x - ans_x
            midpoint_y = -(midpoint_y - ans_y)

            ans_x = 0.0
            ans_y = 0.0

            angle = 0.0

            if midpoint_y < 0:
                angle = 360.0-np.arccos(np.inner([1, 0], [midpoint_x, midpoint_y])/np.linalg.norm([midpoint_x, midpoint_y]))*180.0/np.pi
            else:
                angle = np.arccos(np.inner([1, 0], [midpoint_x, midpoint_y])/np.linalg.norm([midpoint_x, midpoint_y]))*180.0/np.pi

            angle = round(angle-90.0, 2) # -90 to convert angle to be from y axis. aruco code gives angle wrt x axis, but controller (probably) expects wrt y axis i.e. yaw
            # self.get_logger().info(str(angle))
            ArucoMarkerAngles[i] = angle

        # returning the angles of the ArUco markers in degrees as a dictionary
        return ArucoMarkerAngles

    def calibrateCenter(self, ArucoDetailsDict):
        '''
        Purpose:
        ---
        Calibrates center of arena by finding mid point of 3 aruco markers( tl, tr, br ) because "bl" isnt reliably detected

        Input Arguments:
        ---
        self:ArUcoDetector
        ArucoDetailsDict:List

        Returns:
        ---
        List: [centerX, centerY] Calibrated values of arena's center

        Example call:
        ---
        arenaCenter = self.calibrateCenter(ArucoDetailsDict)
        '''
        #returns board center
        tl = ArucoDetailsDict[8][0]
        tr = ArucoDetailsDict[10][0]
        br = ArucoDetailsDict[12][0]

        centerX = tl[0] + (tr[0] - tl[0])/2.0
        centerY = tr[1] + (br[1] - tr[1])/2.0
        # msg = "centerX " + str(centerX) + " centerY " + str(centerY)

        # msg = f'{tl[0]} {tl[1]}  {tr[0]} {tr[1]}  {br[0]} {br[1]}'
        # msg = f'{tl[0]} {tr[0]} {centerX}'

        # self.get_logger().info(msg)

        return [centerX, centerY]

    def mark_ArUco_image(self, image, ArucoDetailsDict, ArucoCorners):
        '''
        Purpose:
        ---
        Helper function to draw aruco marker ID's, rotation of the marker and path of bot

        Input Arguments:
        ---
        self:ArUcoDetector
        image: Image captuered by gazebo camera 
        ArucoDetailsDict: Dictionary containing aruco marker ids and center
        ArucoCorners: Dictionary containing corners of detected aruco markers

        Returns:
        ---
        image: Image with the extra details overlayed

        Example call:
        ---
        img = self.mark_ArUco_image(img, ArucoDetailsDict, ArucoCorners)
        '''
        for ids, details in ArucoDetailsDict.items():
            center = details[0]
            center = [int(center[0]), int(center[1])]
            thickn = 1
            cv2.circle(image, center, thickn, (0, 0, 255), -1)

            corner = ArucoCorners[int(ids)]
            cv2.circle(image, (int(corner[0][0]), int(corner[0][1])), thickn, (50, 50, 255), -1)
            # cv2.circle(image, (int(corner[1][0]), int(corner[1][1])), thickn, (0, 255, 0), -1)
            # cv2.circle(image, (int(corner[2][0]), int(corner[2][1])), thickn, (128, 0, 255), -1)
            # cv2.circle(image, (int(corner[3][0]), int(corner[3][1])), thickn, (25, 255, 255), -1)
            
            
            for index, item in enumerate(self.bot_path): 
                if index == len(self.bot_path) -1:
                    break
                cv2.line(image, item, self.bot_path[index + 1], [0, 255, 0], 2)

            tl_tr_center_x = int((corner[0][0] + corner[1][0]) / 2)
            tl_tr_center_y = int((corner[0][1] + corner[1][1]) / 2)

            cv2.line(image, center, (tl_tr_center_x,
                     tl_tr_center_y), (255, 0, 0), thickn)
            display_offset = int(
                math.sqrt((tl_tr_center_x - center[0])**2+(tl_tr_center_y - center[1])**2))
            cv2.putText(image, str(ids), (center[0]-int(display_offset/2)-5,
                        center[1]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), thickn)
            angle = details[1]
            cv2.putText(image, str(
                angle), (center[0]-display_offset, center[1]+10), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
        return image

    def publishBotLocation(self, botLocation):
        '''
        Purpose:
        ---
        Publishes Bot's location and yaw

        Input Arguments:
        ---
        self:ArUcoDetector
        botLocation:List location of the bot in the format [x, y, theta]

        Returns:
        ---
        None

        Example call:
        ---
        self.publishBotLocation([botCenterX, botCenterY, botTheta])
        '''
        #botLocation is in the format [x, y, theta]
        botTwist = Pose2D()
        botTwist.x = botLocation[0]
        botTwist.y = -botLocation[1] #idk why but '-' is required
        botTwist.theta = math.radians(botLocation[2])

        # msg = "cal" + str(round(botLocation[0], 2)) + " " + str(round(botLocation[1], 2)) + " " + str(round(botLocation[2], 2))
        # self.get_logger().info(msg)

        self.publisher.publish(botTwist)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
