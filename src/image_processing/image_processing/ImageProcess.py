import cv2 # OpenCV
import numpy as np
from scipy import ndimage
import os
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

"""
Used to process an image and determine the location of apples in the image.
Follows the same algorithm used in the matlab code.
Can be tested by running this file
"""

def processImage(image, imageNum=0):
    # Split the RGB channels of the image
    B, G, R = cv2.split(image)

    # r and g are used in a logical statement to determine if the pixel is an apple or not
    rgb_sum = cv2.divide(cv2.add(cv2.add(R, G), B), 2)

    r = np.where(R > rgb_sum, 1, 0)
    g = np.where(G > rgb_sum, 1, 0)


    # D1 and D2 are the equations used to determine if apple or not
    D1 = 0.09 * r - 0.13 * g      # D1 = r && !g
    D2 = 0.1 * r - 0.08 * g - 0.02        # D2 = 
    

    # This statement goes through all the pixels and if the condition is met
    # the pixel gets a 1 showing that it is an apple 0 if it isn't
    redObjectsMask0 = np.logical_and(D1 > 0, D2 > 0).astype(np.uint8)

    # Save RedFiltered Image
    cv2.imwrite(f'~/images/RedFiltered_{imageNum}.png', r*255)

    # Save Original Image
    cv2.imwrite(f'~/images/OriginalImage_{imageNum}.png', image)


    # The below commands erode the image and makes the apples smoother...
    # Assuming redObjectsMask0 is a binary image
    # Create the size of the eroder
    structuringElement = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # Erode the image
    redObjectsMask1 = cv2.erode(redObjectsMask0, structuringElement)

    # Close the morphology
    redObjectsMask2 = cv2.morphologyEx(redObjectsMask1, cv2.MORPH_CLOSE, structuringElement)

    # Set smallest "apple" acceptable
    smallestAcceptableArea = 6000

    # ------------------- Mark the "non-apple" holes ---------------------------------#
    # Find all contours in the binary image
    contours, _ = cv2.findContours(redObjectsMask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out small contours based on area
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > smallestAcceptableArea]

    # Create an empty mask to draw the filtered contours
    redObjectsMask3 = np.zeros_like(redObjectsMask2)

    # Draw the filtered contours on the mask
    cv2.drawContours(redObjectsMask3, filtered_contours, -1, (255), thickness=cv2.FILLED)


    # --------------------- Fill the non-apple holes ---------------------------------#
    redObjectsMask4 = ndimage.binary_fill_holes(redObjectsMask3).astype(np.uint8)

    # Save RedObjectsMask0 Image
    cv2.imwrite(f'~/images/RedObjectsMask0_{imageNum}.png', redObjectsMask0 * 255)

    # Save RedObjectsMask4 Image
    cv2.imwrite(f'~/images/RedObjectsMask4_{imageNum}.png', redObjectsMask4 * 255)

    
    # Create a version of the last mask that can be viewed
    viewing_mask = cv2.multiply(redObjectsMask4, 255)
    viewing_mask = cv2.cvtColor(viewing_mask, cv2.COLOR_GRAY2RGB)

    # Assuming redObjectsMask4 is a binary image
    # Find all contours in the binary image
    contours, _ = cv2.findContours(redObjectsMask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If there are apples within the field of view
    if len(contours) != 0:

        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the index of the largest contour
        largest_contour_index = contours.index(largest_contour)

        # Find and return the center of all apples
        apple_centroids = [(float(cv2.moments(contour)['m10'] / cv2.moments(contour)['m00']), float(cv2.moments(contour)['m01'] / cv2.moments(contour)['m00'])) for contour in contours]

        # Draw Blue Box around all apples
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            viewing_mask = cv2.rectangle(viewing_mask, (x, y), (x + w, y + h), (255, 0, 0), 2)
        
        # Draw Red Box around biggest apple
        x, y, w, h = cv2.boundingRect(largest_contour)
        viewing_mask = cv2.rectangle(viewing_mask, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
        
    else:
        apple_centroids = None
        largest_contour_index = None
    
    # Create a list of geometry msg points
    apple_points = []
    if not apple_centroids is None:
        for centroid in apple_centroids:
            point = Point()
            point.x = centroid[0]
            point.y = centroid[1]
            point.z = 0.0
            apple_points.append(point)

    return largest_contour_index, apple_points, viewing_mask


if __name__ == "__main__":
    image_path = os.path.expanduser('~/ws_orbot/TestPics/apple2.JPG')  # Replace with your image path
    cv_bridge = CvBridge()
    image = cv2.imread(image_path)
    index, apple_points, mask = processImage(image)

    print(f'Index of largest apple: {index}')
    print(f'Apple points: {apple_points}')
    
    # Display the image
    cv2.imshow('Apples', mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
