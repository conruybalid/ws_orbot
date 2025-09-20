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

### Saving Images

This node imports the save_image method as defined below and uses it in each of the subscriber callbacks. To enable or disable image saving, simply comment or uncomment the save_image line 
```python 
save_image(cv_image, "<filename>")
```
in each callback:

```python
def arm_image_callback(self, msg):
        self.get_logger().debug('Received an arm rgb image')
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        save_image(cv_image, "Arm_Image") # Saving the image

        cv_image = self.Resize_to_screen(cv_image)
        self.images[0] = cv_image
```


# Other Functions

## VideoQueue.py

This file contains a class used in video_publisher that handles the video stream and guarantees that it has access to the latest frame.

## ZedClicker.py

This script was used to test the ZED camera and get 3D coordinates from a mouse click on the color image.

## save_image.py

This file contains a method used to save an image file. It takes the image and desired filename for the image as arguments and currently saves them to ```~/Desktop/Images/\<date\>```. Each folder and image file is named by date it was saved.



