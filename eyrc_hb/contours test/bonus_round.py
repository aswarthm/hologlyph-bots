import math
import cv2
import numpy as np

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

def on_change(val):
    global contour_num
    contour_num = val
    print(contour_num)
    cv2.setTrackbarMax("start index", "image", l[val])
    on_change2(len(contours[contour_num]))

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
    else:
        ctr1 = contours[i][int(s[i]):l[i]] 
        ctr2 = contours[i][:int(s[i]+f*l[i]-l[i])]
        cv2.drawContours(output_img, ctr1, -1, (0,0,255), 1)
        cv2.drawContours(output_img, ctr2, -1, (0,0,255), 1)
        
        for contour_array in [ctr1, ctr2]:
            list = [contour.tolist() for contour in contour_array if len(contour) > 0]
            contours_list.extend(list)



    cv2.imshow("image", output_img)

def on_select_button_click(state, x, y, flags, params):
    if state == cv2.EVENT_LBUTTONDOWN:
        with open("saved_contour.txt", "a") as file:
                    if contours_list is not None:
                        for point in contours_list:
                            file.write(f"{point}\n")
        print("added to file")

cv2.imshow("image", output_img)
cv2.createTrackbar('contour#', "image", 0, N, on_change)
 
cv2.setMouseCallback("image", on_select_button_click)

cv2.waitKey(0)
cv2.destroyAllWindows()
