import cv2
import numpy as np
import os
from datetime import datetime

def save_image(image, fileName: str):
    # Create the directory path with the current date
    date_str = datetime.now().strftime("%Y-%m-%d")
    save_path = os.path.join(os.path.expanduser("~"), "Desktop", "Images", date_str)
    os.makedirs(save_path, exist_ok=True)

    # Define the file name and save the image
    file_name = os.path.join(save_path, f'{fileName}_{datetime.now().strftime("%H-%M-%S")}.jpg')
    cv2.imwrite(file_name, image)
    print(f"Image saved to {file_name}")

# Example usage:
if __name__ == "__main__":
    # Capture image from the default camera
    red_image = (255, 0, 0) * np.ones((480, 640, 3), dtype=np.uint8)
    save_image(red_image, "TestImage")