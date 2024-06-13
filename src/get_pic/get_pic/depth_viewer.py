import cv2

# Replace 'rtsp://your_stream_url' with the actual RTSP stream URL
rtsp_url = 'rtsp://192.168.1.10/depth'

# Create a VideoCapture object to read the RTSP stream
cap = cv2.VideoCapture(rtsp_url,cv2.CAP_GSTREAMER)

# Check if the VideoCapture object was successfully opened
if not cap.isOpened():
    print('Failed to open RTSP stream')
    exit()

# Loop to continuously read frames from the RTSP stream
while True:
    # Read a frame from the RTSP stream
    ret, frame = cap.read()

    # Check if the frame was successfully read
    if not ret:
        print('Failed to read frame from RTSP stream')
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame , cv2.COLOR_BGR2RGBA)

    # Display the grayscale frame (depth map)
    cv2.imshow('Depth Map' , frame)

    # Check if the 'q' key was pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close the OpenCV windows
cap.release()
cv2.destroyAllWindows()