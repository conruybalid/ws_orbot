# Nodes

## arm_location_service

This node creates a service that accepts an emtpy call and returns the relative location of the largest apple seen by the arm camera. In the process, it publishes a viewable version of the red mask it used to /masked_image_topic

## zed_location_service

This node creates a service that accepts an emtpy call and returns the absolute location (with respect to the arm base) of the largest apple seen by the zed camera. In the process, it publishes a viewable version of the red mask it used to /zed_mask_topic


# Other Functions

## ImageProcess.py

This function is used to process the image from the arm camera. It applies a red mask and identifies the location of the apple.
