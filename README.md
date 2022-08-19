# Real-Time-3D-reconstruction-with-Kimera-VIO

This markdown file explains the steps to generate a 3D real-time mesh reconstruction using Intel Realsense depth camera and Kimera. [Kimera](https://github.com/MIT-SPARK/Kimera) is a C++ library for real-time metric-semantic visual Simultaneous Localizaiton and Mapping (SLAM). Kimera consists of four components: Visual Inertial Odometry ([Kimera-VIO])(https://github.com/MIT-SPARK/Kimera-VIO), mesh module reconstruction ([Kimera-Mesher](https://github.com/MIT-SPARK/Kimera-VIO)), robust pose graph optimization ([Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO)), and 3D semantic segmentaion ([Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)). Please, refer to their about [paper](https://arxiv.org/pdf/1910.02490.pdf) to read more about how all these key components work together. It is a lightweight, robust and efficient library that works on CPU. 


Tested with **ROS Noetic** on **Ubuntu 20.04 LTS**. 


#### Intel Realsense Camera

Kimera uses stereo images and IMU (if available) as an input. [RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) is depth camera by Intel. 

<img src="file:///home/ibrahimov/Downloads/Real-Time-3D-reconstruction-with-Kimera-VIO/imgs/d435_camera_modules.jpg"
     alt="Camera Modules" width="640" height="330"/>



1. Install or update camera driver?! 
2. Install ros wrapper of the Realsense camera 



#### Kimera

1. Architecture and main modules (insert diagram here)
2. Kimera ROS architecture and message passing diagram 


#### Kimera VIO and Mesher

1. Install ROS wrapper for Kimera 
2. 
2. Build and test a sample rosbag
3.  Using Euroc dataset?!

3. Get input from the camera and 3D reconstruct in real-time 
