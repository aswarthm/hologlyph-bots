#!/usr/bin/env python3

'''
improve by calculating corners only once instead of calibrating everytime
'''

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
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
from std_msgs.msg import Bool

# Import the required modules
##############################################################


isSimulator = True


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
        ##disable#self.get_logger().info("feedback start")

        self.bot_ids = [1, 2, 3]

        self.pubs = {}
        self.penDownSubscriptions = {}
        self.cv_bridge = CvBridge()

        self.penDownCallbacks = [self.penDown_1, self.penDown_1, self.penDown_2, self.penDown_3]
        self.isPenDown = [False, False, False, False]

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(arucoDict, parameters)

        # Subscribe the topic /camera/image_raw
        if(isSimulator):
            self.subscription = self.create_subscription(sensor_msgs.msg.Image,
                                                        "/camera/image_raw",
                                                        self.image_callback,
                                                        10)
        else:
            self.subscription = self.create_subscription(sensor_msgs.msg.Image,
                                                        "/image_raw",
                                                        self.image_callback,
                                                        10)
                                                    

        
        for i in self.bot_ids:
            self.pubs[i] = self.create_publisher(Pose2D,
                                                f"pen{i}_pose",
                                                10)

            self.penDownSubscriptions[i] = self.create_subscription(Bool,
                                                        f"pen{i}_down",
                                                        self.penDownCallbacks[i],
                                                        10)

        self.bot_path = {}
        for i in self.bot_ids:
            self.bot_path[i] = []


    def penDown_1(self, msg):
        self.isPenDown[1] = msg.data
        self.get_logger().info("Bot 1 pen Down: " + str(msg.data))

    def penDown_2(self, msg):
        self.isPenDown[2] = msg.data
        self.get_logger().info("Bot 2 pen Down: " + str(msg.data))

    def penDown_3(self, msg):
        self.isPenDown[3] = msg.data
        self.get_logger().info("Bot 3 pen Down: " + str(msg.data))
    
    def undistort(self, img):

        camera_matrix = np.array([431.68922,   0., 328.16933,
                                  0., 431.88023, 222.03144,
                                  0.,   0.,   1.]).reshape((3, 3))

        dist_coeffs = np.array(
            [-0.354009, 0.106389, -0.000579, -0.001064, 0.000000]).reshape(-1)

        width = 640
        height = 480
        undistorted_image = cv2.undistort(
            img, camera_matrix, dist_coeffs, None  # , newcameramatrix
        )
        return undistorted_image

    
    def perspective_transform(self, img):
        pts1 = np.float32([[122, 11], [519, 9], [125, 418], [522, 414]])
        # Size of the Transformed Image
        pts2 = np.float32([[0, 0], [500, 0], [0, 500], [500, 500]])

        # for val in pts1:
        # cv2.circle(undistorted_image,(val[0],val[1]),5,(0,255,0),-1)
        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(img, M, (500, 500))
        return dst

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

        # Detect Aruco marker
        DetectedArucoMarkers = {}
        ArucoDetailsDict = {}
        ArucoCorners = {}
        ArucoMarkerAngles = {}

        if(isSimulator):
            sharpen_kernel = np.array([  # this works better
                [0, -1, 0],
                [-1, 6, -1],
                [0, -1, 0]
            ])
            img_sharp = cv2.filter2D(img, -1, sharpen_kernel)
            corners, ids, rejected = self.arucoDetector.detectMarkers(img_sharp)
            for i in range(0, ids.shape[0]):
                DetectedArucoMarkers[int(ids[i][0,])] = corners[i][0,]
        else:
            img = self.undistort(img)
            img = self.perspective_transform(img)
            img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            corners, ids, rejected = self.arucoDetector.detectMarkers(img)
            for i in range(0, ids.shape[0]):
                DetectedArucoMarkers[int(ids[i][0,])] = corners[i][0,]

        # sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])

        # img_sharp = img_2
        # img = img_2

        # corners, ids, rejected = self.arucoDetector.detectMarkers(img_2)
        '''
            id 1    bot
            id 4    bott left
            id 8    top left
            id 10   top right
            id 12   bott right
        '''

        ArucoMarkerAngles = self.getOrientationDeg(DetectedArucoMarkers)

        for i in range(0, ids.shape[0]):
            center = np.mean(corners[i][0], axis=0)
            ArucoDetailsDict[int(ids[i][0,])] = [
                center, ArucoMarkerAngles[int(ids[i][0,])]]
            ArucoCorners[int(ids[i][0,])] = corners[i][0,]

        # img = cv2.aruco.drawDetectedMarkers(image = img, corners = corners, ids=ids, borderColor=(0, 255, 0))

        # ##disable#self.get_logger().info(str(rejected))
        # img = self.mark_ArUco_image(img, self.DetectedArucoMarkers, self.DetectedArucoMarkers)

        # ##disable#self.get_logger().info(str(ids))

        # Publish the bot coordinates to the topic  /detected_aruco
        try:
            arenaCenter = self.calibrateCenter(ArucoDetailsDict)
        except:
            pass
            # self.get_logger().info("Corner Aruco Markers Not Detected")

        for i in self.bot_ids:
            try:

                ArucoDetailsDict[i][0][0] += 10.0 #pen offset
                ArucoDetailsDict[i][0][1] += 10.0 #pen offset
                

                bot_loc = ArucoDetailsDict[i] #get bot id's details
                # msg = "aru" + str(round(bot_loc[0][0], 2)) + "\t" + str(round(bot_loc[0][1], 2)) + "\t" + str(round(bot_loc[1], 2))
                # ##disable#self.get_logger().info(msg)#################################################verify if bot coords is same as gazebo coords
                # msg =  "cur" + str(round(self.hb_x, 2)) + "\t" + str(round(self.hb_y, 2)) + "\t" + str(round(self.hb_theta*180.0/math.pi, 2))
                # ##disable#self.get_logger().info(msg)



                #skipping theta calibration, by assuming it doesnt matter, calibrate if required
                botCenterX = float(bot_loc[0][0] - arenaCenter[0]) + 250.0
                botCenterY = float(bot_loc[0][1] - arenaCenter[1]) + 250.0
                botTheta = float(bot_loc[1]+00.0)  ##############debug change this value later, do not hardcode values. 4.24 because aruco marker is not exactly 0degress to baase of bot

                self.bot_path[i].append( (int(bot_loc[0][0]), int(bot_loc[0][1]), int(self.isPenDown[i])) )

                # msg =  "cal" + str(round(botCenterX, 2)) + " " + str(round(botCenterY, 2)) + " " + str(round(bot_loc[1], 2))
                
                self.publishBotLocation(i, [botCenterX, botCenterY, botTheta]) # bot_id, [x, y, theta]
            except:
                pass
                # self.get_logger().info("Bot Aruco Marker Not Detected")

        # except KeyError as e:
        #     #bot id not found or corner ids not found
        #     #either publish panic message to stop bot or just pass, assume id will be detected in next loop
        #     #better to stop bot as it will prevent it from rolling out of arena
        #     # ##disable#self.get_logger().error(str(e.message))
        #     ##disable#self.get_logger().error("Some Aruco Tags Were Not Detected")
        # # except Exception as e:
        # #     ##disable#self.get_logger().error(str(e))
        #     pass

        img = self.mark_ArUco_image(img, ArucoDetailsDict, ArucoCorners)
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
            
            angle = round(angle-90.0, 2) % 360.0 # -90 to convert angle to be from y axis. aruco code gives angle wrt x axis, but controller (probably) expects wrt y axis i.e. yaw
            
            if(angle > 180.0):
                angle = angle - 360 # 355 -> -5
            angle = round(angle, 2)

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
        bl = ArucoDetailsDict[4][0]
        tl = ArucoDetailsDict[8][0]
        tr = ArucoDetailsDict[10][0]
        br = ArucoDetailsDict[12][0]

        corners = np.array([bl, tl, tr, br])
        centerX = corners.mean(axis=0)[0] #- 250.0
        centerY = corners.mean(axis=0)[1] #- 250.0

        # centerX = tl[0] + (tr[0] - tl[0])/2.0
        # centerY = tr[1] + (br[1] - tr[1])/2.0
        # msg = "centerX " + str(centerX) + " centerY " + str(centerY)

        # msg = f'{tl[0]} {tl[1]}  {tr[0]} {tr[1]}  {br[0]} {br[1]}'
        # msg = f'{tl[0]} {tr[0]} {centerX}'

        # ##disable#self.get_logger().info(msg)

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
            
        bot_cols = {
            1: [0, 0, 255],
            2: [255, 0, 0],
            3: [0, 255, 0],
        }
        for i in self.bot_ids:
            path = self.bot_path[i]
            for index, item in enumerate(path):
                if index == len(path) - 1:
                    break
                if(item[2]):
                    cv2.line(image, item[:2], path[index + 1][:2], bot_cols[i], 2)
                else:
                    cv2.line(image, item[:2], path[index + 1][:2], [50, 50, 50], 1)
                    
        return image

    def publishBotLocation(self, bot_id, botLocation):
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
        botTwist.y = botLocation[1] #idk why but '-' is required
        botTwist.theta = math.radians(botLocation[2])

        # msg = str(bot_id) +  "cal" + str(round(botLocation[0], 2)) + " " + str(round(botLocation[1], 2)) + " " + str(round(botLocation[2], 2))
        # self.get_logger().info(msg)

        self.pubs[bot_id].publish(botTwist)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()