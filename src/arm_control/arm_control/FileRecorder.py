import os
from datetime import datetime
import csv
import time

class FileRecorder():
    def __init__(self):
        # Create the directory path with the current date
        date_str = datetime.now().strftime("%Y-%m-%d")
        save_path = os.path.join(os.path.expanduser("~"), "Desktop", "PositionData", date_str)
        os.makedirs(save_path, exist_ok=True)

        # Define the file name and save the image
        fileName = "orbotLocation"
        row = ['time', 'X', 'Y', 'Z']
        self.file_name = os.path.join(save_path, f'{fileName}_{datetime.now().strftime("%H-%M-%S")}.csv')
        with open(self.file_name, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)

        self.start_time = time.time()


    def RestartTimer(self):
          self.start_time = time.time()

    def getElaspedTime(self):
         return float(time.time() - self.start_time)
        
    def RecordPosition(self, position):
          with open(self.file_name, 'a', newline='') as file:
              writer = csv.writer(file)
              writer.writerow([self.getElaspedTime(), position[0], position[1], position[2]])



if __name__ == '__main__':
    timer = FileRecorder()
    timer.RestartTimer()
    timer.RecordPosition([2,3,4])