import cv2
import os

# RTSP stream URL
rtsp_url = "rtsp://192.168.1.10/color"

# Create a VideoCapture object
cap = cv2.VideoCapture(rtsp_url)

# Check if the stream is opened successfully
if not cap.isOpened():
    print("Failed to open RTSP stream")
    exit()

# Read and display frames from the stream
while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("RTSP Stream", frame)
        
        #cv2.waitKey(0)

        filename = os.path.expanduser("~/image.jpg")
        cv2.imwrite(filename, frame)
        break
        
    if not ret:
        print("Failed to read frame from RTSP stream")
        break

    cv2.imshow("RTSP Stream", frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break



# Release the VideoCapture object and close the window
cap.release()
cv2.destroyAllWindows()