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

"""
When loading the model from the internet, it will attempt to uninstall torch/torchvision
 and install the "correct versions". This will not work ideally on the Jetson.

 Check the README for instructions to install the versions of torch and torchvision that work with the Orin GPU
"""

class AI_model:
    def __init__(self, model_path='./src/image_processing/resource/87Epochbest.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path) # DO NOT USE FORCE_RELOAD

    def GetAppleCoordinates(self, image: np.ndarray, confidence_threshold: float=0.5) -> List[Tuple[int, int, int, int, float]]:
        """
        This function takes an image as input and return a list of tuples containing [x1, y1, x2, y2] (box corners)
          sorted by confidence of the detected apples
        """
        results = self.model(image) # Get the results from the model

        apple_pixels = results.xyxy[0] # apple_pixels auto sorted by confidence

        apple_coordinates = [
            (int(x1.item()), int(y1.item()), int(x2.item()), int(y2.item()), conf.item()) 
            for x1, y1, x2, y2, conf, *_ in apple_pixels 
            if conf.item() > confidence_threshold]

        return apple_coordinates # [Tuple[x1, y1, x2, y2, confidence], ...] top left and bottom right corners of the box and confidence
    

def main():

    model = AI_model()
    image = cv2.imread('TestPics/apples.JPG')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print(f'Apples: {model.GetAppleCoordinates(image, confidence_threshold=0.5)}')
    result = model.model(image)
    result.show()
    

if __name__ == '__main__':
    main()