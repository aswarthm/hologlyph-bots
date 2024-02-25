import cv2
import numpy as np

# from skimage.morphology import skeletonize
# from skimage import io
import matplotlib.pyplot as plt
import json

def Edge_Detection_Canny(cv_image):
    cv_image =cv2.flip(cv_image ,1)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    # customize the second and the third argument, minVal and maxVal
    # in function cv2.Canny if needed
    get_edge = cv2.Canny(blurred, 10, 100)
    cv_image = np.hstack([get_edge])
    cv2.imshow('Corners_Extracted', cv_image)
    cv2.waitKey(0)
    return cv_image



def showImg(img):
    cv2.imshow('img', img) 
    cv2.waitKey(0)  


def on_change2(val):
    global s, f, contours_list
    if contour_num == 1:
        val = 115
    elif contour_num == 2:
        val = 66   
    elif contour_num == 3:
        val = 65   
    elif contour_num == 4:
        val = 151
        f = 1/2.5
    elif contour_num == 5:
        val = 504
        f = 1/1.95
    elif contour_num == 6:
        val = 312
    elif contour_num == 7:
        val = 5
    elif contour_num == 8:
        val = 33
    elif contour_num == 9:
        val = 28
    elif contour_num == 10:
        val = 26
    elif contour_num == 11:
        val = 213
    elif contour_num == 12:
        val = 117
    elif contour_num == 13:
        val = 294
        f = 1/1.88
    elif contour_num == 14:
        val = 144
        f = 1/1.88
    elif contour_num == 15:
        val = 606
        f = 1/1.9
    elif contour_num == 16:
        val = 152
    elif contour_num == 17:
        val = 89
    elif contour_num == 18:
        val = 47
        f = 0.6
    elif contour_num == 19:
        val = 50
        f = 0.6
    elif contour_num == 20:
        val = 193
        f = 0.52
    elif contour_num == 21:
        val = 197
        f = 0.52
    elif contour_num == 22:
        val = 471 
    elif contour_num == 23:
        val = 234
    elif contour_num == 24:
        val = 889

    

    # print(val)
    s[contour_num] = val
    output_img = np.zeros((500, 500,3), np.uint8)
    cv2.drawContours(output_img, contours, -1, (100,100,100), 1)
    i = contour_num

    if(s[i]+f*l[i] < l[i]):
        ctr = contours[i][int(s[i]):int(s[i]+f*l[i])]
        cv2.drawContours(output_img, ctr, -1, (0,0,255), 1)
        for contour_array in [ctr]:
            list = [contour.tolist() for contour in contour_array if len(contour) > 0]
            contours_list.extend(list)
            print(f"{list}\n\n")
    else:
        ctr1 = contours[i][int(s[i]):l[i]] 
        ctr2 = contours[i][:int(s[i]+f*l[i]-l[i])]
        cv2.drawContours(output_img, ctr1, -1, (0,0,255), 1)
        cv2.drawContours(output_img, ctr2, -1, (0,0,255), 1)

        li = []
        for contour_array in [ctr1, ctr2]:
            list = [contour.tolist() for contour in contour_array if len(contour) > 0]
            li.extend(list)
            contours_list.extend(list)
        print(f"{li}\n\n")


    showImg(output_img)
    # cv2.imshow("image", output_img)

output_img = np.zeros((500, 500,3), np.uint8)

img = cv2.imread("elephant.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_, bwimg = cv2.threshold(img, 125, 255, cv2.THRESH_BINARY)
# bwimg=cv2.bitwise_not(bwimg)
cv2.imshow("image", bwimg)
cv2.waitKey(0)
contours, hierarchy = cv2.findContours(bwimg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

N = len(contours)
contour_num = 0
l = [len(contours[i]) for i in range(N)]
s = [0 for i in range(N)]
f = 0.5
contours_list = []
# img = Edge_Detection_Canny(image)
# showImg(img)

# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# ret,thresh = cv2.threshold(gray,50,255,0)

# showImg(gray)


# # ret,thresh = cv2.threshold(gray,50,255,0)
# # showImg(thresh)

# edged = cv2.Canny(gray, 30, 200)

# showImg(edged)

# img_bgr = image
# img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV_FULL)

# # Filter out low saturation values, which means gray-scale pixels(majorly in background)
# bgd_mask = cv2.inRange(img_hsv, np.array([0, 0, 0]), np.array([255, 30, 255]))

# # Get a mask for pitch black pixel values
# black_pixels_mask = cv2.inRange(img_bgr, np.array([0, 0, 0]), np.array([70, 70, 70]))

# # Get the mask for extreme white pixels.
# white_pixels_mask = cv2.inRange(img_bgr, np.array([230, 230, 230]), np.array([255, 255, 255]))

# final_mask = cv2.max(bgd_mask, black_pixels_mask)
# final_mask = cv2.min(final_mask, ~white_pixels_mask)
# final_mask = ~final_mask
# erode = 5
# final_mask = cv2.erode(final_mask, np.ones((erode, erode), dtype=np.uint8))
# showImg(final_mask)
# dialate = 3
# final_mask = cv2.dilate(final_mask, np.ones((dialate, dialate), dtype=np.uint8))
# # final_mask = cv2.ximgproc.thinning(final_mask, thinningType=cv2.ximgproc.THINNING_GUOHALL)

# showImg(final_mask)
# showImg(~final_mask)

# contours,hierarchy = cv2.findContours(~final_mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

print(len(contours))

image = bwimg

# for i, cnt in enumerate(contours):
#     color = list(np.random.random(size=3) * 120 + 90)
#     cv2.drawContours(image, contours, i, color, 1)
#     showImg(image)

for i in range(1, 25):
    contour_num = i
    on_change2(len(contours[contour_num]))

with open("./contours.txt", "+w") as file:
    for contour in contours:
        file.write(str(contour.tolist()))
        file.write("\n\n\n\n\n\n\n\n\n\n")

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

