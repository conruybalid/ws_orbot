import pyzed.sl as sl
import argparse
import math
import cv2
import numpy as np



parser = argparse.ArgumentParser()
parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
parser.add_argument('--ip_address', type=str, help='IP Adress, in format a.b.c.d:port or a.b.c.d, if you have a streaming setup', default = '')
parser.add_argument('--resolution', type=str, help='Resolution, can be either HD2K, HD1200, HD1080, HD720, SVGA or VGA', default = '')
opt = parser.parse_args()


def parse_args(init):
    if len(opt.input_svo_file)>0 and opt.input_svo_file.endswith(".svo"):
        init.set_from_svo_file(opt.input_svo_file)
        #print("[Sample] Using SVO File input: {0}".format(opt.input_svo_file))
    elif len(opt.ip_address)>0 :
        ip_str = opt.ip_address
        if ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4 and len(ip_str.split(':'))==2:
            init.set_from_stream(ip_str.split(':')[0],int(ip_str.split(':')[1]))
            #print("[Sample] Using Stream input, IP : ",ip_str)
        elif ip_str.replace(':','').replace('.','').isdigit() and len(ip_str.split('.'))==4:
            init.set_from_stream(ip_str)
            #print("[Sample] Using Stream input, IP : ",ip_str)
        #else :
            #print("Unvalid IP format. Using live stream")
    if ("HD2K" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD2K
        #print("[Sample] Using Camera in resolution HD2K")
    elif ("HD1200" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1200
        #print("[Sample] Using Camera in resolution HD1200")
    elif ("HD1080" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD1080
        #print("[Sample] Using Camera in resolution HD1080")
    elif ("HD720" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.HD720
        #print("[Sample] Using Camera in resolution HD720")
    elif ("SVGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.SVGA
        #print("[Sample] Using Camera in resolution SVGA")
    elif ("VGA" in opt.resolution):
        init.camera_resolution = sl.RESOLUTION.VGA
        #print("[Sample] Using Camera in resolution VGA")
    #elif len(opt.resolution)>0: 
        #print("[Sample] No valid resolution entered. Using default")
    #else : 
        #print("[Sample] Using default resolution")


init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.ULTRA,
                                 coordinate_units=sl.UNIT.MILLIMETER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)

zed = sl.Camera()
status = zed.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()


# Create a ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.sdk_verbose = 1 # Enable verbose logging
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Set the depth mode to performance (fastest)
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use meter units

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    print("Error {}, exit program".format(err)) # Display the error
    exit()

    # Capture 50 images and depth, then stop
i = 0
image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()
runtime_parameters = sl.RuntimeParameters()
while i < 50:
    # Grab an image
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns sl.ERROR_CODE.SUCCESS
        zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud
        i = i + 1

# Get and print distance value in mm at the center of the image
# We measure the distance camera - object using Euclidean distance
width = image.get_width()
height = image.get_height()
x = round(image.get_width() / 2)
y = round(image.get_height() / 2)
err, point_cloud_value = point_cloud.get_value(x, y)
# distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
#                      point_cloud_value[1] * point_cloud_value[1] +
#                      point_cloud_value[2] * point_cloud_value[2])
# print("Distance to Camera at ({0}, {1}): {2} mm".format(x, y, distance), end="\r")  

# Retrieve the image, depth, and point cloud
zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
zed.retrieve_measure(depth, sl.MEASURE.DEPTH) # Retrieve depth matrix. Depth is aligned on the left RGB image
zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) # Retrieve colored point cloud

# Convert the image to HSV color space
hsv_image = cv2.cvtColor(image.get_data(), cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds for the red color in HSV
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Create a mask for the red color
mask = cv2.inRange(hsv_image, lower_red, upper_red)

# Find the contours of the red mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the center of the largest contour
if len(contours) > 0:
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])

    # Get the depth value at the center of the mask
    depth_value = depth.get_value(center_x, center_y)
    x, y, z, something = point_cloud.get_value(center_x, center_y)[1]

    print(f'point cloud: {x}, {y}, {z}, {something}')

    # # Convert the center coordinates to meters
    x_distance = (-x + 402) / 1000
    y_distance = (y + 420) / 1000

    print(f'x_distance: {x_distance}, y_distance: {y_distance}')

   
    depth_value = depth_value[1] - 200 # Get the depth value in mm
    depth_value = depth_value/1000 # Convert the depth value to meters
    print("Depth at center of red mask: {} m".format(depth_value))
    # print("Distance from left of the image (x-axis): {} meters".format(x_distance))
    # print("Distance from top of the image (y-axis): {} meters".format(y_distance))
else:
    print("No red mask found.")
