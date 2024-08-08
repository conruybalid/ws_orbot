# Nodes

## video_publisher

This node gets color images from the arm camera and publishes it to the following topic:
- /image_topic - rgb image


## zed_publisher

This node gets information from the zed camera and publishes the information to respective topics:
- /zed_image_topic - rgb image
- /zed_depth_topic - depth image
- /zed_pointcloud_topic - xyz coordinate array

## zed_subsciber - debug node

This node is used to test the topic outputs of zed_publisher

## kortex_vision_subscriber.py

This node is used to test the topic outputs of the kortex_vision package

## videoSubscriber - debug node

This node is used to display image from the following topics in a concise manner:
- /image_topic
- /camera/depth/image_raw
- /masked_image_topic
- /zed_image_topic
- /zed_mask_topic

If all the topics are active, then it will display the color and depth images from the arm and zed camera, as well as the masked images used whenever service calls are made


# Other Functions

## VideoQueue.py

This file contains a class used in video_publisher that handles the video stream and guarantees that it has access to the latest frame.

## ZedClicker.py

This script was used to test the ZED camera and get 3D coordinates from a mouse click on the color image.


