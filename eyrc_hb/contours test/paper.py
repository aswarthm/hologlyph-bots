from Tfg_nodes.srv import * #Import the package
import rospy
83
#Librarys for the image proccesing and transport
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math #Math library for calculus of distances
import matplotlib.pyplot as plt #Library to plot the points
#######################################
# FUNTIONS FOT THE FACE DETECTION ALGORITH
########################################
def resize_image(image):
image_r =image.copy()
[rows,cols, lenght]=np.shape(image)
i=0.5
down_width = cols
while down_width > 500:
down_width = int(cols*i) #y cols
down_height = int(rows*i) #x rows
down_points = (down_width, down_height)
image_r = cv2.resize(image, down_points, interpolation=
cv2.INTER_LINEAR)
i= i/2
return image_r
def draw_found_faces(detected, image, color):
print "Drawing faces..."
for (x, y, width, height) in detected:
cv2.rectangle( image, (x-50, y-50), (x + width+ 50, y +
height+ 50), color, thickness=2)
image_c = image[y-50:y + height+ 50 ,x-50:x + width+
50 ]
return image_c
return image

def Face_detection_funtion(image):
image_wr=resize_image(image)
# Convert image to grayscale
image = cv2.cvtColor(image_wr, cv2.COLOR_BGR2GRAY)
84
# Create Cascade Classifiers
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades +
"haarcascade_frontalface_default.xml")
profile_cascade = cv2.CascadeClassifier(cv2.data.haarcascades
+ "haarcascade_profileface.xml")
# Detect faces using the classifiers
detected_faces = face_cascade.detectMultiScale(image=image,
scaleFactor=1.3, minNeighbors=4)
detected_profiles =
profile_cascade.detectMultiScale(image=image, scaleFactor=1.3,
minNeighbors=4)
# Filter out profiles
profiles_not_faces = [x for x in detected_profiles if x not
in detected_faces]
# Draw rectangles around faces on the original, colored image
image_c = draw_found_faces(detected_faces, image_wr, (0, 255,
0)) # RGB - green
return image_c
#########################################
# FUNTIONS FOR THE GREEDY ALGORITHM
#########################################
#Definition of classes
class Point: #Point definition
def __init__(self, x, y, v , dist):
self.x = x #Coordinate x
self.y = y #Coordinate y
self.v = v #Vertex (1: yes ; 2: no)
self.dist = dist
def printP(self): #Print the points
print('x: ' + str(self.x) + ', y:' + str(self.y) + ',
v:'+ str(self.v) + ', dist:'+ str(self.dist) )
def calcDist(self, val2): #Distance from given point to other
a = (val2.x-self.x)
b = (val2.y-self.y)
return math.sqrt(a*a+b*b) #Pitagora's theorem
def CN(self): #Looks for the closest neighbor
85
aux = float('inf') #Each time function is called,
distance is supposed as infinity
pos = 0 #Initialitation of index; also used for the last
point
for i in range(len(points)): #Checks the distance with
all the points
dist = self.calcDist(points[i])
if (dist > 0) & (dist < aux): #Keeps the point
with minor distance
aux = dist
pos = i
points[i].dist=dist
p=points[pos]
points.pop(pos)
return p
#Draw the points
abcisas = [] #X
ordenadas = [] #Y
distances =[]
abcisasaux = [] #X
ordenadasaux = [] #Y
distancesaux =[]
def Path_Planning_with_Greddy_Colours(points):
#Starts looking for the closest neighbour(CN)
FP = points[0]
FCN = FP.CN()
for k in range(len(points)): #Looks for the CN of each point
FP.x = float('inf') #Turns the points to infinity to
not taking later
FP.y = float('inf')
FP.v = 0
FP = FCN #Take the CN as the starting point
FCN = FP.CN() #Calculate the CN
if FP.dist > 4:
abcisasaux.append(FP.x)
ordenadasaux.append(FP.y)
distancesaux.append(FP.dist)
abcisas.append(FP.x)
ordenadas.append(FP.y)
distances.append(FP.dist)
86
plt.plot(abcisas,ordenadas ,color='blue') #Plot the graph of
points
plt.plot(abcisasaux,ordenadasaux , color='red') #Plot the
graph of point
plt.show()
points = []
def Path_Planning_Creation(cv_image):
[rows,cols]=np.shape(cv_image)
rospy.loginfo("The rows \n%s" , rows)
rospy.loginfo("The cols \n%s" , cols)
#Create the arrays with the coordinate of the point that
belongs to the corners detected
for i in range(rows-1):
for j in range(cols-1):
if cv_image[i,j] == 255:
nx = j
ny = i
nv = 1
dist = float('inf')
points.append(Point(nx, ny, nv ,dist))
Path_Planning_with_Greddy_Colours(points)
cv2.imshow('Final Result', cv_image)
cv2.waitKey(0)
######################################################
# FUNTIONS FOT THE PROCESSING IMAGE ALGORITHM
########################################
def Edge_Detection_Canny(cv_image):
cv_image =cv2.flip(cv_image ,1)
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)
# customize the second and the third argument, minVal and
maxVal
# in function cv2.Canny if needed
get_edge = cv2.Canny(blurred, 10, 100)
cv_image = np.hstack([get_edge])
cv2.imshow('Corners_Extracted', cv_image)
cv2.waitKey(0)
87
return cv_image
Result =image_processing_msg()
def Image_process(image):
try:
print "Procesing the image..."
bridge =CvBridge()
cv_image =bridge.imgmsg_to_cv2(image.Image , "bgr8")
cv2.imshow('Image_Received',cv_image)
cv2.waitKey(0)
#Here goes the camera processing code that will return
and arrays of points
print "Detecting faces..."
face_Detection=Face_detection_funtion(cv_image)
print "Detecting Corners with Edge_Detection_Canny..."
Corners_detected= Edge_Detection_Canny(face_Detection)
print "Creating Path_Planning ..."
Path_Planning_Creation(Corners_detected)
[rows,cols,lengt]=np.shape(face_Detection)
Result.x = abcisas
Result.y = ordenadas
Result.Dist = distances
Result.Cols=cols
Result.Rows=rows
print "Process finished correctly..."
return image_processing_msgResponse(Result.x , Result.y
, Result.Dist , Result.Cols, Result.Rows)
except rospy.ServiceException, e:
print "Image processing failed: %s"%e
except CvBridgeError, err:
print "Image processing failed: %s"%err
def main_server():
88
rospy.init_node('Image_p_server')
#Name of the node for the service, name of the type of msg
s =rospy.Service('procesing',image_processing_msg,
Image_process)
print "Waiting for an image"
rospy.spin()
print "\nClosing the server"
if __name__ == "__main__":
main_server()
