# AruCo Detection

[AruCo Markers](https://nusit.nus.edu.sg/technus/ar-tags-and-their-applications-in-computer-vision-tasks/) is similar to a QR Code which is used for various Computer Vision application. 

Some applications include: 
* Camera Calibration
* Pose Estimation
* Navigation

### Tools/Packages Required:
1. [OpenCV](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-20-04/)
2. [ROS](http://wiki.ros.org/ROS/Installation)
3. [C++](https://linuxconfig.org/how-to-install-g-the-c-compiler-on-ubuntu-20-04-lts-focal-fossa-linux)


### Running the Code:

After you have all the tools/packages installed, make sure you add the following to `.bashrc` file:
```shell
export TURTLEBOT3_MODEL="waffle_pi"
```

Clone this repository in your catkin workspace and copy the models in `AruCoDetection/gazebo_models` to `~/.gazebo/models`.

To launch the AruCo World in Gazebo run:
```shell
roslaunch aruco_detection aruco_world.launch
```

To start the AruCo Marker detection node run:
```shell
rosrun aruco_detection aruco_node
```

To Tele-operate the turtlebot3:
```shell
rosrun aruco_detection tcbot_node
```

Turtlebot can be teleoperated using the arrow keys and stopped using the Space bar.


<p align="center">
<img src="/images/aruco_gif.gif" alt="AruCo Marker" style="width:500px;" align="center"/>
</p>
