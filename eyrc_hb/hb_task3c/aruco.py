import cv2
import numpy as np
import time
import math

vid = cv2.VideoCapture(2) # check the number it can change according to hardware
time.sleep(1)
img, ret = vid.read()
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, parameters)

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
corners, ids, rejected = arucoDetector.detectMarkers(img_sharp)
'''
    id 1    bot
    id 4    bott left
    id 8    top left
    id 10   top right
    id 12   bott right
'''
for i in range(0, ids.shape[0]):
    DetectedArucoMarkers[int(ids[i][0,])] = corners[i][0,]

ArucoMarkerAngles = getOrientationDeg(DetectedArucoMarkers)

for i in range(0, ids.shape[0]):
    center = np.mean(corners[i][0], axis=0)
    ArucoDetailsDict[int(ids[i][0,])] = [
        center, ArucoMarkerAngles[int(ids[i][0,])]]
    ArucoCorners[int(ids[i][0,])] = corners[i][0,]

# img = cv2.aruco.drawDetectedMarkers(image = img, corners = corners, ids=ids, borderColor=(0, 255, 0))
img = mark_ArUco_image(img, ArucoDetailsDict, ArucoCorners)

def getOrientationDeg(DetectedArucoMarkers):
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


def mark_ArUco_image(image, ArucoDetailsDict, ArucoCorners):
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