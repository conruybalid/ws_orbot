import cv2
import threading
import time

class VideoStreamHandler:
    """
    Class to handle video stream from a camera
    Continuously read frames from the camera and stores the latest frame
    """
    def __init__(self, rtsp_url, gstreamer=False):
        self.rtsp_url = rtsp_url
        if not gstreamer:
            self.cap = cv2.VideoCapture(rtsp_url)
        else:
            self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_GSTREAMER)
        self.latest_frame = None
        self.running = True
        self.read_thread = threading.Thread(target=self.update_frame)
        self.read_thread.start()

    def update_frame(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
            else:
                print("Failed to read frame")
                break

    def get_latest_frame(self):
        return self.latest_frame

    def stop(self):
        self.running = False
        self.read_thread.join()
        self.cap.release()



def main(): # Usage
    rtsp_url = "rtsp://192.168.1.10/color"
    video_stream_handler = VideoStreamHandler(rtsp_url)

    try:
        while True:
            time.sleep(0.5)  # Simulate time between processing events
            frame = video_stream_handler.get_latest_frame()
            if frame is not None:
                cv2.imshow("Latest Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("No frame available")
    finally:
        video_stream_handler.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()