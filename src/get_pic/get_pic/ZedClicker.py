import pyzed.sl as sl
import cv2

"""
This script was used to test the ZED camera and get 3D coordinates from a mouse click on the color image.
"""


# Initialize the ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
zed.open(init_params)

# Function to get 3D coordinates from pixel
def get_3d_coordinates(x, y):
    point_cloud = sl.Mat()
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
    err, point_cloud_value = point_cloud.get_value(x, y)
    if err == sl.ERROR_CODE.SUCCESS:
        return point_cloud_value[:3]  # Returns (X, Y, Z)
    return None

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        coords = get_3d_coordinates(x, y)
        if coords:
            print(f"3D Coordinates: {coords}")

# Set up OpenCV window and mouse callback
cv2.namedWindow("ZED")
cv2.setMouseCallback("ZED", mouse_callback)

# Main loop
while True:
    image = sl.Mat()
    zed.retrieve_image(image, sl.VIEW.LEFT)
    cv2.imshow("ZED", image.get_data())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
zed.close()
cv2.destroyAllWindows()
