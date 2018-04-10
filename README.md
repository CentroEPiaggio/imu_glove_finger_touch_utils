# PISA IIT IMU GLOVE FINGER TOUCH UTILS

A ROS package with nodes for working with the IMU Glove provided for the PISA IIT SoftHand. A Standard score algorithm is used for identifying finger touches using the measurements from the IMUs on the glove.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Indigo or newer and the qb_interface_node package. 

Clone this in you catkin workspace and build:
INSERT NMMI link here.

### Installing

To install this package just clone into your catkin_ws and catkin_make.

## Running the finger collision identification node

To launch the finger_collision_identification node use the following launch instruction:

```
roslaunch imu_glove_finger_touch_utils launchCollisionIdentification.launch
```

To get the norms of the accelerations of the imus on a topic use the following launch instruction:

```
roslaunch imu_glove_finger_touch_utils launchNormPublisher.launch
```
