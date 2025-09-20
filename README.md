# Running the Current System

Use a the peripheral_launch launch file to start
- video_publisher node (arm camera)
- zed_publisher node (zed camera)
- arm_location_service node (service to identify apples from arm camera)
- zed_location_service node (service to identify apples from zed camera)
- video_subscriber node (both cameras) (used for debug)
- arm_move_action (action node to move the arm)
```console
ros2 launch launch/peripheral_launch.py
```

In a new terminal, start the master_node node (begin pick apple routine)
```console
ros2 run masters master_node
```

Don't forget to source each new terminal with
```console
source install/setup.bash
```

## Saving Images

For research purposes, you may want to save the images captured by the robot. Image saving is handled by the video_subscriber node in the get_pics package and is saved in ```~/Desktop/Images/<date>```. To enable or disable image saving for each image type, uncomment or commment the ```save_image(cv_image, \<filename\>)``` line in each callback function in ```src/get_pic/get_pic/videoSubscriber.py```. More info can be found in ```src/get_pic/get_pic/README.md``` 

# Using the Workspace
## Building Packages

After making any changes to the source files, the package must be rebuilt using colcon build. This will create the build, install and log files

Always do this in a separate command window than where you run the nodes

### Build all Packages

in ws_orbot
```console
colcon build
```
### Build One Package

in ws_orbot
```console
colcon build --packages-select <package_name>
```

## Source the Overlay

In order for a terminal to use everything in the workspace, it must first be sourced. If ros2 is not recognizing custom nodes or msg types, it is likely because it was not sourced first

in ws_orbot
```console
source install/setup.bash
```

this must be run in every new terminal

## Run a Node

in ws_orbot
```console
ros2 run <package_name> <node_name>
```
example:
```console
ros2 run masters master_node
```

use control-c to kill the node at any time

## Launch Files
Launch files can contain multiple nodes to launch at the same time.
These are located in the launch folder

in ws_orbot
```console
ros2 launch <path/to/launch/file>
```
example:
```console
ros2 launch launch/debuglaunch.py
```

## Debug tools
### Nodes
To show active nodes
```console
ros2 node list
```
### Topics
#### To show active topics
```console
ros2 topic list
```

#### To listen to a topic

```console
ros2 topic echo </topic_name>
```

#### To publish to a topic continuously

```console
ros2 topic pub </topic_name> <msg_type> '<args>'
```

#### To publish to a topic once

```console
ros2 topic pub --once </topic_name> <msg_type> '<args>'
```
example:
```console
ros2 topic pub --once /zed_distance_topic std_msgs/msg/Float32 '{data: 0.85}'
```

#### To see topic details

```console
ros2 topic info </topic_name>
```

### Actions

same as Topics, using ros2 action instead

### Interfaces

messages, services, and actions

#### Show Inteface details

```console
ros2 interface show <msg_type>
``` 
examples:
```console
ros2 interface show custom_interfaces/action/MoveArm
```
and
```console
ros2 interface show custom_interfaces/msg/ArmControl
```

### Graphing
#### To create a graph of the current system
```console
rqt_graph
```

# Creating your Own Stuff

## Create a Package
in ws_orbot/src
```console
ros2 pkg create --build-type <ament_type> --license <license> <package_name>
```
example:
```console
ros2 pkg create --build-type ament_python --license Apache-2.0 my_super_cool_package
```

## Create a Node
in ws_orbot/src/<package_name>/<package_name>:

create a python file with this basic setup
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
  def __init__(self):
    super().__init__('my_node')
    
    # Create publishers and/or subscribers

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

# Dependencies

This system was designed on Ubuntu 20.04 using the ROS2 Foxy Distribution

The Kinova Robotic Arm uses the Kortex Python API, which can be downloaded from https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md

The Zed Camera requires the ZED SDK Library, which can be downloaded from https://www.stereolabs.com/developers/release 
