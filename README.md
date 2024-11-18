# RealSense CPP Library

Wrapper library for the official Intel RealSense SDK. 

Motivation: While developing my ROS2 autonomy stack, I struggled with high latency due to large RGBD communication. I replaced the ROS2 communication from camera to any nodelet by implementing a wrapper library in C++. This reduced all the communication issues I had. While developing it, I could not find clean sources of code already available. This is why I am sharing it here. It's not intended to be a wrapper with lots of functions integrated for general purposes. You can use it as it is for RGBD data, or extend it as your task requires. Feel free to make pull requests if you want to contribute.

Functionality: Initializes camera with custom configuration and passes at every call to the ```grabFrames``` function the latest pair of color (RGB) and depth (D) information.
A basic ROS2 nodelet is implemented to test the library and as example of its usage.

## Install Intel RealSense SDK and dependencies
```
./install_realsense.sh
```

## ROS2 testing
Note: this is not stricly needed for the purpose of the wrapper. 
Feel free to remove ROS2 dependencies and only use the hpp library.

### Create workspace
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### Build & source
```
colcon build --symlink-install
```
```
source install/local_setup.bash
```

### Launch & visualize
```
ros2 launch realsense-cpp rs_camera.launch.py
```
```
ros2 run rqt_image_view rqt_image_view
```