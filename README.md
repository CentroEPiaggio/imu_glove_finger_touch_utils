# PISA IIT IMU GLOVE FINGER TOUCH UTILS

A ROS package with nodes for working with the IMU Glove provided for the PISA IIT SoftHand. A Standard score algorithm is used for identifying finger touches using the measurements from the IMUs on the glove.

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Indigo or newer and the qb_interface_node package. 

Clone this in you catkin workspace and build:
https://github.com/NMMI/IMU

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

### Notes on Identification Algorithm

Most of the parameters of the identification algorithm can be changed from the `config_col.yaml` file which is already loaded by the `launchCollisionIdentification` launch file. A short description of each parameter is given in the yaml file itself.

N.B: These parameters are tuned optimally for the Centro Piaggio setup with the KUKA LWR and Ocado version of the PISA/IIT SoftHand. When using it on another setup please tune the params properly.

## Issues with finger touch identification

The algorithm actually identifies the peaks correctly both when the hand is still and is touched by an object on any finger and also while the hand closes if the touch actually creates a peak. 

The problem is that there might not be an actual peak in the accelerometer signals. So, the work for a better identification of touches is still in progress.