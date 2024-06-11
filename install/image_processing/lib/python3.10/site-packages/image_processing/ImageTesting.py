import cv2 # OpenCV
import numpy as np
from scipy import ndimage
import os
from cv_bridge import CvBridge



def main():
    # Your code here
    image_path = os.path.expanduser('~/apple.jpg')  # Replace with your image path

    cv_bridge = CvBridge()
    image_original = cv2.imread(image_path)

    image = cv_bridge.cv2_to_imgmsg(image_original)

    image_cv = cv_bridge.imgmsg_to_cv2(image)
    
    # Split the RGB channels of the image
    R, G, B = cv2.split(image_cv)


if __name__ == "__main__":
    main()