import cv2
import numpy as np

from skimage.morphology import skeletonize
from skimage import io
import matplotlib.pyplot as plt



def showImg(img):
    cv2.imshow('img', img) 
    cv2.waitKey(0)  

image = cv2.imread('image_mode.png')

# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# ret,thresh = cv2.threshold(gray,50,255,0)

# showImg(gray)


# # ret,thresh = cv2.threshold(gray,50,255,0)
# # showImg(thresh)

# edged = cv2.Canny(gray, 30, 200)

# showImg(edged)

img_bgr = image
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV_FULL)

# Filter out low saturation values, which means gray-scale pixels(majorly in background)
bgd_mask = cv2.inRange(img_hsv, np.array([0, 0, 0]), np.array([255, 30, 255]))

# Get a mask for pitch black pixel values
black_pixels_mask = cv2.inRange(img_bgr, np.array([0, 0, 0]), np.array([70, 70, 70]))

# Get the mask for extreme white pixels.
white_pixels_mask = cv2.inRange(img_bgr, np.array([230, 230, 230]), np.array([255, 255, 255]))

final_mask = cv2.max(bgd_mask, black_pixels_mask)
final_mask = cv2.min(final_mask, ~white_pixels_mask)
final_mask = ~final_mask
erode = 5
final_mask = cv2.erode(final_mask, np.ones((erode, erode), dtype=np.uint8))
showImg(final_mask)
dialate = 3
final_mask = cv2.dilate(final_mask, np.ones((dialate, dialate), dtype=np.uint8))
# final_mask = cv2.ximgproc.thinning(final_mask, thinningType=cv2.ximgproc.THINNING_GUOHALL)

showImg(final_mask)
showImg(~final_mask)

contours,hierarchy = cv2.findContours(~final_mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

print(len(contours))

for i, cnt in enumerate(contours):
    color = list(np.random.random(size=3) * 120 + 90)
    cv2.drawContours(image, contours, i, color, 3)
showImg(image)

# for cnt in contours:
#    print(len(cnt))
# #    print(cnt)
#    print("\n\n")
#    approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
#    (x,y)=cnt[0,0]

#    if len(approx) >= 5:
#       img = cv2.drawContours(image, [approx], -1, (0,255,255), 3)
#       cv2.putText(img, 'Polygon', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)  
#       showImg(img)


