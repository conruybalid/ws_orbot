import torch
import cv2
import numpy as np
from typing import List, Tuple

import sys

"""
This class is a wrapper for the YOLOv5 model used to detect apples in an image.
It uses the ultralytics/yolov5 repository to load the model
    The first time this is done, internet connection is required
    The model is then saved locally in a cache and can be loaded without internet connection
    It is saved in ~/.cache/torch/hub/ultralytics_yolov5_master
Then, an image can be passed to the model to get the coordinates of the apples in the image
"""


class AI_model:
    def __init__(self, model_path='./src/image_processing/resource/87Epochbest.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

    def GetAppleCoordinates(self, image: np.ndarray, confidence_threshold=0.5):
        """
        This function takes an image as input and return a list of tuples containing [x1, y1, x2, y2] (box corners)
          sorted by confidence of the detected apples
        """
        results = self.model(image)

        apple_pixels = results.xyxy[0] # apple_pixels auto sorted by confidence

        apple_coordinates = [
            (int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item()), conf.item()) 
            for x1, y1, x2, y2, conf, *_ in apple_pixels 
            if conf.item() > confidence_threshold]

        return apple_coordinates
    

def main():

    model = AI_model()
    image = cv2.imread('TestPics/apples.JPG')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print(f'Apples: {model.GetAppleCoordinates(image, confidence_threshold=0.5)}')
    result = model.model(image)
    result.show()
    

if __name__ == '__main__':
    main()