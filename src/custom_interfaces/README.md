This package contains custom msg, service, and action definitions used in this project.

# Interfaces

## Messages

### ArmControl

This is a message that contains all the information needed to move the arm

```
position
    x
    y
    z

angle
    x
    y
    z

reference_frame # 2 for gripper, 3 for base of arm

gripper_state # 0 for no change, 1 to open, 2 to close
```

## Services

### GetLocation

The is the service call used to request the location of an apple
```
# no call msg
---
uint8 error_status  # 0: success, 1: no apple found, 2: Processing Error, 3: No Image
geometry_msgs/Point apple_coordinates
```

## Actions

### MoveArm

This action is used by the arm_move_action. The goal is of type ArmControl msg

```
goal
    position
        x
        y
        z

    angle
        x
        y
        z

    reference_frame # 2 for gripper, 3 for base of arm

    gripper_state # 0 for no change, 1 to open, 2 to close

---
result # True or False
---
feedback # feedback string
```