# irob_vision_support

This package supports for stereo vision used for surgical automation. See the yet interfaced devices, and hints for those usage and camera calibration below. Before interfacing the cameras, the *irob_autosurg* package should be compiled, see [here](https://github.com/ABC-iRobotics/irob-autosurg/blob/master/README.md).

## Usage with USB webcam pair

Stereo cameras, consists of a pair of USB webcameras can be interfaced this way using the package:

Install packages *ros-kinetic-cv-camera* and *ros-kinetic-camera-calibration*

    sudo apt install ros-kinetic-cv-camera
    sudo apt install ros-kinetic-camera-calibration
    
The video feed can be started by the launch file *stereo_cam_usb_raw.launch* (device ID is based on the connection order of the cameras), for example:
    
    roslaunch irob_vision_support stereo_cam_usb_raw.launch device_id_left:=0 device_id_right:=1

    
Before usage the stereo camera pair should be calibrated by the following command:

    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/saf/stereo/right/image_raw left:=/saf/stereo/left/image_raw right_camera:=/saf/stereo/right left_camera:=/saf/stereo/left --approximate=0.1

Read on in section **Calibration**
  

## Stereo endoscope through Blackmagic frame grabber

In most of the dVRK labs, the stereo video feed from the da Vinci is connected to PC using a frame grabber from [Blackmagic Design](https://www.blackmagicdesign.com/). The interfacing of the freame grabber can be seen as follows.

### Desktop Video

To be able to use the frame grabber, download and install the following packages from the [Blackmagic homepage](https://www.blackmagicdesign.com/support/family/capture-and-playback):

- desktopvideo
- desktopvideo-gui
- mediaexpress

After installation, launch Blackmagic Desktop Video Setup and set all connectors to PAL and single. Then Media Express can be launched to view or capture video.

### Acquire video stream in ROS

The video stream can be interfaced into ROS using the ROS package [gscam](https://github.com/ros-drivers/gscam). This package can also be installed using apt, but this version supports gstreamer 0.10. The Black magic card requires the the subset *plugins-bad*, whom 0.10 version cannot be installed on Ubuntu 16.04 LTS. So, the best workaround is to clone the gscam package from GitHub and build it with the flag for gstreamer 1.0 turned on.

First, install gstreamer dependencies, see on [gscam github](https://github.com/ros-drivers/gscam):

        sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer-tools  libgstreamer-plugins-bad1.0-dev

After the dependencies are installed, clone and build the gscam ROS package from GitHub with the flag for gstreamer 1.0:
 
    cd catkin_ws/src
    git clone https://github.com/ros-drivers/gscam.git
    cd gscam
    cd ../..
    catkin config --cmake-args -DGSTREAMER_VERSION_1_x=On

Then, in decklink.launch replace the following to ensure compatibility vith gstreamer 1.0:

    decklinksrc -> decklinkvideosrc
    ffmpegcolorspace -> videoconvert

To remove interlace from the stream, replace the line for gscam_config in decklink.launch to

    <param name="gscam_config" value="decklinkvideosrc mode=$(arg MODE) connection=$(arg CONNECTION) subdevice=$(arg SUBDEVICE) ! videoconvert ! deinterlace ! videoconvert"/>

Test the video stream:

    roslaunch gscam decklink.launch MODE:=pal SUBDEVICE:=1
    
The video feed can be started by the launch file *stereo_cam_blackmagic.launch*, for example:
    
    roslaunch irob_vision_support stereo_cam_blackmagic_raw.launch
    
This launch file includes a pair of preprocessing nodes, those are simply averaging two successive frames to reduce noise. Before usage the stereo camera pair should be calibrated by the following command:

    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.025 right:=/saf/stereo/right/image_raw left:=/saf/stereo/left/image_raw right_camera:=/saf/stereo/right left_camera:=/saf/stereo/left --approximate=0.1

Useful commands for debug if something goes wrong:

    gst-inspect-1.0
    gst-inspect-1.0 decklinkvideosrc



## Usage with a pair of Raspberry Pies

Two Raspberry Pies with cameras connected can also be used to acquire stereo feed. For this purpose, connect the Raspberries with the PC over ethernet. The network should be configured somehow like this example:

  - Set up a fix ip on all devices
  - Append this lines to the file */etc/hosts*: 
  
        192.168.1.104		master
        192.168.1.105		left-pi
        192.168.1.106		right-pi
        
  - Export ROS_HOSTNAME, ROS_IP, ROS_MASTER_URI on all devices
  - For further help, see:
      - http://wiki.ros.org/ROS/NetworkSetup
      - http://elinux.org/RPi_Setting_up_a_static_IP_in_Debian
      

Then, install and launch Raspicam nodes on the Raspberries: https://github.com/UbiquityRobotics/raspicam_node, and launch *stereo_cam_raspberry* on the PC. Calibration and usage is similar to the USB cams.

  
## Camera calibration
Camera calibration is quite easy in ROS using the package *camera_calibration*. The example calibration command can be seen in the guide for different cameras, but here are some important things that should be kept in mind:
- Stereo camera calibrations must be saved to separate files for each camera (elsehow the disparity will be extremely noisy)
- Use the calibration app with option --approximate=0.1
- ROS Jade and earlier are having issues with calibration, better to use Kinetic!

## View the stereo stream

The launch files *stereo_cam_blackmagic.launch* will launch a [stereo_image_proc](http://wiki.ros.org/stereo_image_proc) node as well, that rectifies the images, and calculates disparity map based on the camera calibration. The video streams can be displayed by the ROS node [rqt_image_view](http://wiki.ros.org/rqt_image_view), that can be launched by tiping:

    rosrun rqt_image_view rqt_image_view
    
The disparity image can be seen using a node [disparity_view](http://wiki.ros.org/image_view), for example:

    rosrun image_view disparity_view image:=/irob-saf/stereo/disparity

## Registration of the arms to the camera

In order to the arms be able to work in the coordinate system of the camera, those coordinate systems should be registered. It can be done by the Matlab script dvrk_camera_registration in the folder matlab, using a checkerboard marked fixed on the instrument. After started the computer vision nodes and the dVRK console, simply move the arm to differet positions and click on the window generated by Matlab each time. After enough points are measured, the results will be saved to the specified registration file.

## Implement your algorithms in Python

You have to enable execution on all Python files before try running them using the command:

    chmod +x python_node.py

## Useful links
* http://wiki.ros.org/cv_camera
* http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
* http://wiki.ros.org/stereo_image_proc
* https://github.com/UbiquityRobotics/raspicam_node


