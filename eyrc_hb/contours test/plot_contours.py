import cv2
import numpy as np
import ast
import time

# Create a blank image
height, width = 500, 500  # adjust these values to match your contours
image = np.zeros((height, width, 3), dtype=np.uint8)

# Read contours from file
with open("./contours.txt", "r") as file:
    contours_str = file.read().split("\n\n\n\n\n\n\n\n\n\n")

# Parse contours and draw them onto the image
for contour_str in contours_str:
    if contour_str:  # skip empty strings
        # Parse the string into a list
        contour_list = ast.literal_eval(contour_str)

        # Convert the list to a numpy array and reshape it to match the original shape
        contour = np.array(contour_list).reshape((-1, 1, 2)).astype(np.int32)

        # Draw the contour onto the image
        # cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)

        #draw contour point by point, animating the drawing
        # Draw contour point by point, animating the drawing
        # print(contour)
        for point in contour:
            point = point[0]
            print(point)
            x = point[0]
            y = point[1]
            cv2.circle(image, (int(x), int(y)), 1, (0, 255, 0), -1)
            cv2.imshow("Image", image)
            cv2.waitKey(1)

        # for i in range(0, len(contour)):
        #     # Get the start and end points of the line
        #     start_point = contour[i - 1][0]
        #     end_point = contour[i][0]

        #     # Calculate the number of pixels based on the distance between the points
        #     distance = int(np.hypot(end_point[0] - start_point[0], end_point[1] - start_point[1]))

        #     for j in range(distance):
        #         # Interpolate the position
        #         x = start_point[0] + (end_point[0] - start_point[0]) * j / distance
        #         y = start_point[1] + (end_point[1] - start_point[1]) * j / distance

        #         # Draw the pixel
        #         cv2.circle(image, (int(x), int(y)), 1, (0, 255, 0), -1)
        #         cv2.imshow("Image", image)
        #         cv2.waitKey(1)

# Display the image
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()