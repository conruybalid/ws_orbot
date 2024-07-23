import torch
import cv2
import numpy as np
from typing import List, Tuple


class AI_model:
    def __init__(self, model_path='./src/image_processing/resource/best.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

    def GetAppleCoordinates(self, image: np.ndarray, confidence_threshold=0.5):
        results = self.model(image)

        apple_pixels: List[Tuple[int, int]] = []
        for *box, conf, cls in results.xyxy[0]: # xyxy, confidence, class 
            if conf.item() > confidence_threshold:
                x_center = (box[0].item() + box[2].item()) / 2 # x = (x1 + x2) / 2
                y_center = (box[1].item() + box[3].item()) / 2 # y = (y1 + y2) / 2
                apple_pixels.append((x_center, y_center))
        
        return apple_pixels
        
        

    

def main():
    model = AI_model()
    image = cv2.imread('TestPics/apples.JPG')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    print(model.GetAppleCoordinates(image, confidence_threshold=0.5))
    result = model.model(image)
    result.show()

if __name__ == '__main__':
    main()