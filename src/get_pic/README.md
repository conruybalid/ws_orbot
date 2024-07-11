# Nodes

## video_publisher

This node gets information from the arm camera and publishes the information to respective topics:
- /image_topic - rgb image
- /depth_topic - FIXME

Currently, the depth_topic is not being published to because our current method of getting a depth frame causes it to become compressed from 16 bits to 8 bits, which makes it practically unusable

## zed_publisher

This node gets information from the zed camera and publishes the information to respective topics:
- /zed_image_topic - rgb image
- /zed_depth_topic - depth image
- /zed_pointcloud_topic - xyz coordinate array

## zed_subsciber - debug node

This node is used to test the topic outputs of zed_publisher

## videoSubscriber - debug node

This node is used to display image from the following topics in a concise manner:
- /image_topic
- /masked_image_topic
- /zed_image_topic
- /zed_mask_topic

If all the topics are active, then it will display the color image from the arm and zed camera, as well as the masked images used whenever service calls are made


# Other Functions

## VideoQueue.py

This file contains a class used in video_publisher that handles the video stream and guarantees that it has access to the latest frame.

