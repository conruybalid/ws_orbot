import torch
import cv2


class AI_model:
    def __init__(self, model_path='./src/image_processing/resource/best.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

    def predict(self, image):
        return self.model(image)
    

def main():
    model = AI_model()
    image = cv2.imread('TestPics/apples.JPG')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    result = model.model(image)
    result.show()

if __name__ == '__main__':
    main()