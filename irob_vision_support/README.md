# irob_vision_support
Support for computer vision used for surgical automation.

# Usage with USB webcam pair
* sudo apt-get installros-kinetic-cv-camera
* sudo apt-get install ros-kinetic-camera-calibration
* device ID is based on the connection order of the cameras

# Usage with a pair of Raspberry Pi-es
* Raspicam node: https://github.com/UbiquityRobotics/raspicam_node
* ROS network setup:
  - fix ip: 192.168.1.104 255.255.255.0 192.168.1.1
  - DNS: 8.8.4.4
  - /etc/hosts: 
  				192.168.1.104	master
				192.168.1.105	left-pi
				192.168.1.106	right-pi
  .* export ROS_HOSTNAME, ROS_IP, ROS_MASTER_URI
  .* see http://elinux.org/RPi_Setting_up_a_static_IP_in_Debian
  
# Camera calibration
* stereo camera calibrations must be saved from separate files (elsehow the disparity will be extremely noisy)
* use the calibration app with option --approximate=0.1

# Useful links
* http://wiki.ros.org/cv_camera
* http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
* http://wiki.ros.org/stereo_image_proc
* https://github.com/UbiquityRobotics/raspicam_node

