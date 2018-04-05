# irob_vision_support
This package supports for stereo vision used for surgical automation. See the yet interfaced devices, and hints for those usage and camera calibration below. Before interfacing the cameras, the *irob_autosurg* package should be compiled, see [here](https://github.com/ABC-iRobotics/irob-autosurg/blob/master/README.md).

## Usage with USB webcam pair

Stereo cameras, consists of a pair of USB webcameras can be interfaced this way using the package:

- Install packages *ros-kinetic-cv-camera* and *ros-kinetic-camera-calibration*

    sudo apt install ros-kinetic-cv-camera
    sudo apt install ros-kinetic-camera-calibration
    
- The video feed can be started by the launch file *stereo_cam_usb_raw* (device ID is based on the connection order of the cameras), for example:
    
    roslaunch irob_vision_support stereo_cam_usb_raw device_id_left:=0 device_id_right:=1
    
- Before usage the stereo camera pair should be calibrated by the following command:

     rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/ias/stereo/right/image_raw left:=/ias/stereo/left/image_raw right_camera:=/ias/stereo/right left_camera:=/ias/stereo/left --approximate=0.1
    
  

## Black magic guide

TODO

# Usage with a pair of Raspberry Pi-es
* Raspicam node: https://github.com/UbiquityRobotics/raspicam_node
* ROS network setup:
  - fix ip: 192.168.1.104 255.255.255.0 192.168.1.1
  - DNS: 8.8.4.4
  - /etc/hosts: 
  	- 192.168.1.104		master
	- 192.168.1.105		left-pi
	- 192.168.1.106		right-pi
  - export ROS_HOSTNAME, ROS_IP, ROS_MASTER_URI
  - see http://elinux.org/RPi_Setting_up_a_static_IP_in_Debian
  
# Camera calibration
* stereo camera calibrations must be saved to separate files (elsehow the disparity will be extremely noisy)
* use the calibration app with option --approximate=0.1
* example: rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/ias/stereo/right/calibrated/image left:=/ias/stereo/left/calibrated/image right_camera:=/ias/stereo/right/calibrated left_camera:=/ias/stereo/left/calibrated --approximate=0.1
* ROS Jade and earlier are having issues with calibration!
* stereo_cam_usb_raw.launch:
   
    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/ias/stereo/right/image_raw left:=/ias/stereo/left/image_raw right_camera:=/ias/stereo/right left_camera:=/ias/stereo/left --approximate=0.1
    
* stereo_cam_usb and stereo_cam_blackmagic:

    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/ias/stereo/preprocessed/right/image_raw left:=/ias/stereo/preprocessed/left/image_raw right_camera:=/ias/stereo/preprocessed/right left_camera:=/ias/stereo/preprocessed/left --approximate=0.1


# Useful links
* http://wiki.ros.org/cv_camera
* http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
* http://wiki.ros.org/stereo_image_proc
* https://github.com/UbiquityRobotics/raspicam_node

# Matlab custom message generation
* Install add-on "Robotics System Toolbox Interface for ROS Custom Messages" (use sudo matlab)
* Type in Matlab console:
  - folderpath = fullfile('catkin_ws','src', 'irob-autosurg')
  - rosgenmsg(folderpath)
* Follow the instructions suggested by rosgenmsg (savepath works only in sudo mode)
* For a more detailed guide see https://www.mathworks.com/help/robotics/ug/create-custom-messages-from-ros-package.html

# Black magic guide

TODO

