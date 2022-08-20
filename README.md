# Real-Time-3D-reconstruction-with-Kimera-VIO

This markdown file explains the steps to generate a 3D real-time mesh reconstruction using Intel Realsense depth camera and Kimera. [Kimera](https://github.com/MIT-SPARK/Kimera) is a C++ library for real-time metric-semantic visual Simultaneous Localizaiton and Mapping (SLAM). Kimera consists of four components: Visual Inertial Odometry ([Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)), mesh module reconstruction ([Kimera-Mesher](https://github.com/MIT-SPARK/Kimera-VIO)), robust pose graph optimization ([Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO)), and 3D semantic segmentaion ([Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)). Please, refer to their about [paper](https://arxiv.org/pdf/1910.02490.pdf) to read more about how all these key components work together. It is a lightweight, robust and efficient library that works on CPU. 

General overview of Kimera is as follows:



<img src="https://raw.githubusercontent.com/ibrahimovnijat/Real-Time-3D-reconstruction-with-Kimera-VIO/test/imgs/kimera_chart.jpeg?token=GHSAT0AAAAAABWHX7GKII7MPN3Q4WSNALSCYYARMFQ"
     alt="Camera Modules" width="680" height="420" class="center"/>


Tested with **ROS Noetic** on **Ubuntu 20.04 LTS**. 


## Intel Realsense Camera

Kimera uses stereo images and IMU (if available) as an input. I used [RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) depth camera to stream stereo data.  

<img src="https://raw.githubusercontent.com/ibrahimovnijat/Real-Time-3D-reconstruction-with-Kimera-VIO/test/imgs/d435_camera_modules.jpg?token=GHSAT0AAAAAABWHX7GK6HL7SFEOQJ6HAUUAYYARGWA"
     alt="Camera Modules" width="640" height="330" class="center"/>

<img src="https://raw.githubusercontent.com/ibrahimovnijat/Real-Time-3D-reconstruction-with-Kimera-VIO/test/imgs/depth-camera-d435_details.jpg?token=GHSAT0AAAAAABWHX7GLZYQ5LJNLNWMPCIKQYYARHLQ"
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

    3. Build 

        ```
        catkin_init_workspace
        cd ..
        catkin_make clean
        catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
        catkin_make install
        ```
    4. Automatically source setup bash script when new shell is launched. 
        ```
        echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```

#### Launch realsense-ros

In order to start a camera node, you need to start ***rs_camera.launch*** file
 
```
roslaunch realsense2_camera rs_camera.launch
```

It will publish several topics depending the on camera type and parameter setup. Typical topics will include raw_image, rbg_image, camera ifnfo, imu data etc. 

For more information about parameter setup and usage, please, visit: [RealSense ROS](https://github.com/IntelRealSense/realsense-ros).


## Kimera VIO and Mesher

There exists a [ROS wrapper]() for Kimera too, which makes it significantly easier to pass message from depth camera to Kimera and visualize the results with RViz. 

Simple diagram showing the structure of the ROS 

[img] ros wrapper diagram goes here...


**Kimera-VIO-ROS installation**

-  Besides ROS Neotic (see [here](http://wiki.ros.org/noetic/Installation/Ubuntu)), also install non-default dependencies for **mesh_rviz_plugins**

	```
	sudo apt-get install ros-melodic-image-geometry ros-melodic-pcl-ros ros-melodic-cv-bridge
	```
- Update package list and install system dependencies

	```
	sudo apt-get install -y --no-install-recommends apt-utils
	sudo apt-get install -y \
	      cmake build-essential unzip pkg-config autoconf \
	      libboost-all-dev \
	      libjpeg-dev libpng-dev libtiff-dev \
	# Use libvtk5-dev, libgtk2.0-dev in ubuntu 16.04 \
	      libvtk6-dev libgtk-3-dev \
	      libatlas-base-dev gfortran \
	      libparmetis-dev \
	      python-wstool python-catkin-tools \
	
	```

- ROS wrapper installation (see [here](https://github.com/MIT-SPARK/Kimera-VIO-ROS#b-kimeravio-ros-wrapper-installation))
	
	```
	# Setup catkin workspace
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin init
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF
	
	catkin config --merge-devel
	
	# Add workspace to bashrc for automatic sourcing of workspace.
	echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

	
	# Clone repo
	cd ~/catkin_ws/src
	# For ssh:
	git clone git@github.com:MIT-SPARK/Kimera-VIO-ROS.git
	# For https:
	# git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git
	
	# Install dependencies from rosinstall file using wstool
	wstool init # Use unless wstool is already initialized
		
	# For ssh:
	wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_ssh.rosinstall
	# For https
	# wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall
	
	# download and update repos:
	wstool update

	```
	Finally, compile wrapper with catkin build
	
	```
	catkin build
	
	source ~/catkin_ws/devel/setup.bash
	```


**Kimera Semantics installation**

- Install system dependencies

```
sudo apt-get install python-wstool python-catkin-tools  protobuf-compiler autoconf
sudo apt-get install ros-noetic-cmake-modules
```

- ROS wrapper installation

```
# Setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/melodic # Change `melodic` to your ROS distro
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

# Add workspace to bashrc.
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Clone repo
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/Kimera-Semantics.git

# Install dependencies from rosinstall file using wstool
wstool init # Use unless wstool is already initialized

# Optionally add Kimera-Semantics to the rosinstall file
# wstool scrape

# For ssh:
wstool merge Kimera-Semantics/install/kimera_semantics_ssh.rosinstall
# For https:
#wstool merge Kimera-Semantics/install/kimera_semantics_https.rosinstall

# Download and update all dependencies
wstool update
```

Then, compile:

```
catkin build kimera_semantics_ros
source ~/catkin_ws/devel/setup.bash
```



## Usage

Before using the real camera, we can test Kimera with Euroc rosbag. In four different terminals, run the fo
llowing commands:

```
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch run_stereo_dense:=true

roslaunch kimera_semantics_ros kimera_semantics_euroc.launch

rviz -d kimera_semantics_euroc.rviz

rosbag play V1_01_easy.bag --clock
```

<video src="[LINK](https://youtu.be/ZswQS5ymVtU)" controls="controls" style="max-width: 730px;">
</video>


<a href="{https://youtu.be/ZswQS5ymVtU}" title="Demo Video"><img src="{https://i9.ytimg.com/vi/ZswQS5ymVtU/mq3.jpg?sqp=CMjcgpgG&rs=AOn4CLDL9djMpEyn_gaeNgFoyIjnPj7Zvg
}" alt="Demo Video" /></a>


