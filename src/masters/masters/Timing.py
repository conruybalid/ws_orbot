import os
from datetime import datetime
import csv
import time

class OrbotTimer():
    def __init__(self):
        # Create the directory path with the current date
        date_str = datetime.now().strftime("%Y-%m-%d")
        save_path = os.path.join(os.path.expanduser("~"), "Desktop", "TimingData", date_str)
        os.makedirs(save_path, exist_ok=True)

        # Define the file name and save the image
        fileName = "orbotTimer"
        row = ['Send wide Camera Request', 'In front of Apple', 'send arm camera request', 'Apple Centered', 'Start pick Routine', 'twist']
        self.file_name = os.path.join(save_path, f'{fileName}_{datetime.now().strftime("%H-%M-%S")}.csv')
        with open(self.file_name, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)

        self.start_time = time.time()
        self.row = [0,0,0,0,0,0]


    def RestartTimer(self):
          self.start_time = time.time()

    def getElaspedTime(self):
         return float(time.time() - self.start_time)
        
    def RecordTime(self, column: int):
          if column < 6:
            self.row[column] = self.getElaspedTime()
          else:
            print('out of range')

    def RecordRow(self):
         with open(self.file_name, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(self.row)


if __name__ == '__main__':
    timer = OrbotTimer()
    timer.RestartTimer()
    timer.RecordTime(0)
    timer.RecordTime(1)
    timer.RecordTime(2)
    timer.RecordTime(3)
    timer.RecordTime(4)
    timer.RecordTime(5)

    timer.RecordRow()