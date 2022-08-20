# Real-Time-3D-reconstruction-with-Kimera-VIO

This markdown file explains the steps to generate a 3D real-time mesh reconstruction using Intel Realsense depth camera and Kimera. [Kimera](https://github.com/MIT-SPARK/Kimera) is a C++ library for real-time metric-semantic visual Simultaneous Localizaiton and Mapping (SLAM). Kimera consists of four components: Visual Inertial Odometry ([Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)), mesh module reconstruction ([Kimera-Mesher](https://github.com/MIT-SPARK/Kimera-VIO)), robust pose graph optimization ([Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO)), and 3D semantic segmentaion ([Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)). Please, refer to their about [paper](https://arxiv.org/pdf/1910.02490.pdf) to read more about how all these key components work together. It is a lightweight, robust and efficient library that works on CPU. 


Tested with **ROS Noetic** on **Ubuntu 20.04 LTS**. 


#### Intel Realsense Camera

Kimera uses stereo images and IMU (if available) as an input. I used [RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) depth camera to stream stereo data.  

<img src="https://raw.githubusercontent.com/ibrahimovnijat/Real-Time-3D-reconstruction-with-Kimera-VIO/test/imgs/d435_camera_modules.jpg?token=GHSAT0AAAAAABXTCNKAAACVN62QWSMZF3W6YYADHTQ"
     alt="Camera Modules" width="640" height="330" class="center"/>


<img src="https://raw.githubusercontent.com/ibrahimovnijat/Real-Time-3D-reconstruction-with-Kimera-VIO/test/imgs/depth-camera-d435_details.jpg?token=GHSAT0AAAAAABXTCNKBK223CQSQ4BKK66VSYYADMYQ"
     alt="Camera Modules" width="640" height="330" class="center"/>

D435 has two stereo cameras with IR projector and RGB camera. Certain models (such as D435i) also have a built-in IMU. 

Intel provides SDK for RealSense cameras, which can be downloaded [here](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide?_ga=2.206885459.1336921430.1660882175-773886595.1660882175). The original page provides step by step instructions to download the SDK for all Operating Systems. It also allows to install a RealSense Viewer application, which allows to connect to camera via USB. You can set parameteres, stream, take images and even update the camera firmware. Additionally, all available drivers for all realsense camera can be found [here](https://dev.intelrealsense.com/docs/firmware-updates).

Nice thing is RelSense cameras have a [ROS wrapper](https://github.com/IntelRealSense/realsense-ros) which makes it easier to integrate easier use to with Kimera. 

#### Install realsense-ros

* It is possible to install deb package realsense2-camera on Ubuntu.
        
    ```
    sudo apt-get install ros-noetic-realsense2-camera
    ```

* You can build it from source.

    1. Create a catkin workspace 
    
        ```
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src/
        ```
    
    2. Clone realsense-ros repository into ~/catkin_ws/src
        
        ```
        git clone https://github.com/IntelRealSense/realsense-ros.git
        
        cd realsense-ros/

        git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

        cd ..

        ```

  
        
#### Kimera

1. Architecture and main modules (insert diagram here)
2. Kimera ROS architecture and message passing diagram 


#### Kimera VIO and Mesher

1. Install ROS wrapper for Kimera 
2. 
2. Build and test a sample rosbag
3.  Using Euroc dataset?!

3. Get input from the camera and 3D reconstruct in real-time 
