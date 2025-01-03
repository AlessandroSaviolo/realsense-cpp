# RealSense C++ Library

A lightweight and efficient wrapper library for the Intel RealSense SDK, designed to minimize latency and provide streamlined access to RGBD data.

## Motivation
While developing a ROS2 autonomy stack, I encountered significant latency issues with large RGBD data transmissions. By replacing ROS2 communication between the camera and nodelets with a custom C++ wrapper library, I successfully mitigated these issues. During development, I noticed a lack of clean and minimalistic code examples for this purpose, which led me to share this library.

This library is intentionally kept simple, focusing on providing RGBD data with minimal overhead. It is not intended as a comprehensive wrapper with extensive functionality. However, it is designed to be easily extended for specific use cases. Contributions are welcome via pull requests.

## Features
- Customizable camera initialization with user-defined configurations.
- On-demand retrieval of the latest RGB (color) and Depth (D) frames through the `grabFrames` function.
- A basic ROS2 nodelet is included as an example and for testing purposes.

---

## Installation
### 1. Install Intel RealSense SDK and Dependencies
Run the following script to install the necessary SDK and dependencies:
```bash
./install_realsense.sh
```

---

## Configuration
The library assumes the presence of a configuration file `rs_config.json` located in the `config` folder. You can generate this file using the following steps:
1. Run the `realsense-viewer` command.
2. Edit the camera settings, such as exposure, projector intensity, etc., to your preferences.
3. Save the configuration to a file named `rs_config.json` and place it in the `config` folder.

---

## ROS2 Testing
> **Note:** ROS2 is not required to use this wrapper. If you do not need ROS2 integration, you can remove ROS2 dependencies and directly use the `.hpp` library.

### 1. Create a Workspace
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### 2. Clone and Build the Package
Build the workspace using `colcon`:
```bash
git clone https://github.com/AlessandroSaviolo/realsense-cpp
```
Source the workspace:
```bash
cd ~/ros_ws
colcon build --symlink-install
source install/local_setup.bash
```

### 3. Launch and Visualize
Launch the ROS2 nodelet:
```bash
ros2 launch realsense_camera rs_camera.launch.py
```
Visualize the output using `rqt_image_view`:
```bash
ros2 run rqt_image_view rqt_image_view
```

---

## Tested Environment
This code was tested on the following setup:
- **Hardware:** NVIDIA Orin 16GB, Intel RealSense 455
- **Librealsense Version:** v2.55.1
- **ROS2 Distribution:** Humble

---

## Contributing
Contributions are encouraged! Feel free to submit pull requests to:
- Extend functionality.
- Improve documentation.
- Fix bugs or issues.

---

## License
This project is licensed under MIT license.

```license
Copyright (c) 2024 AlessandroSaviolo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

---

## Citation

This repository includes code that supports the paper **"Reactive Collision Avoidance for Safe Agile Navigation"**. 
If you use this library in your research or academic projects, please cite the following:
```bibtex
@article{saviolo2024reactive,
  title={Reactive Collision Avoidance for Safe Agile Navigation},
  author={Saviolo, Alessandro and Picello, Niko and Verma, Rishabh and Loianno, Giuseppe},
  journal={arXiv preprint arXiv:2409.11962},
  year={2024}
}
```
