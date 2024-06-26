import cv2

# Replace 'rtsp://your_stream_url' with the actual RTSP stream URL
rtsp_url = 'rtsp://192.168.1.10/depth'

# Create a VideoCapture object to read the RTSP stream
cap = cv2.VideoCapture(rtsp_url,cv2.CAP_GSTREAMER)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('Y','1','6',' '))
# cap.set(cv2.CAP_PROP_FORMAT, cv2.CV_16U)
cap.set(cv2.CAP_PROP_CONVERT_RGB, False) 

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
    #gray = cv2.cvtColor(frame , cv2.COLOR_BGR2RGBA)
    # Calculate the average depth value from the center of the depth image
    height, width = frame.shape
    center_x = width // 2
    center_y = height // 2
    depth_values = frame[center_y-2:center_y+2, center_x-2:center_x+2]
    print(depth_values)
    average_depth = depth_values.mean()

    # Output the average distance
    print(f"Average distance: {average_depth} units")
    # Check if the 'q' key was pressed to exit the loop
    send_depth = average_depth * .350
    print(send_depth)

    # Display the grayscale frame (depth map)
    cv2.imshow('Depth Map' , frame)
    print(frame[135][240])

    # Check if the 'q' key was pressed to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close the OpenCV windows
cap.release()
cv2.destroyAllWindows()