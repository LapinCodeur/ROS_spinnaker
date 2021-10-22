# Spinnaker wrapper for ROS
This is a small python wrapper for Spinnaker to publish color image from a Flir camera to a Topic.
It is a really simple wrapper to avoid having any trouble with future versions.

# Requirements
This version has only been tested on Ubuntu 20.04, Spinnaker SDK 2.5.0.80 and ROS Noetic, but it should work on other version.
To use it, you need to install the Spinnaker SDK and Spinnaker SDK Python.
For Linux, you will find them in this [repo](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/133154514756)

# Using it
Clone this repo in the /src directory inside your workspace and :
```
catkin_make
source devel/setup.bash
roslaunch ROS_spinnaker camera.launch
```
A widow will start with a preview of the camera, the image showing up will take a bit of time before being to the right dimensions.