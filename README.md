# irob-saf

## About

The *iRob Surgical Automation Framework*---`irob-saf`--- is an open-source ROS-based metapackage, built by the Antal Bejczy Center for Intelligent Robotics (iRob), for the aim to support the research of partial automation in robot-assisted surgery.  The packages of the framework implements basic functionalities, usable as universal building blocks in surgical automation, such as infrastructure to implement subtask-level logic, interfacing of stereo cameras, a hierarchic motion library with parameterizable surgemes, and high-level robot control. The framework were built and tested alongside the da Vinci Resarch Kit (dVRK), however it is easily portable to other platforms as well.
**Compatible with dVRK 2.x.**

## Citation

If you use this framework in your reserach, please cite the following paper:


Tamas D. Nagy and Tamas Haidegger, [“A DVRK-based Framework for Surgical Subtask Automation,”](https://www.uni-obuda.hu/journal/Nagy_Haidegger_95.pdf) Acta Polytechnica Hungarica, Special Issue on Platforms for Medical Robotics Research, vol. 16, no. 8, pp. 61–78, 2019.


## List of Packages
* [irob_msgs](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_msgs)
* [irob_utils](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_utils)
* [irob_robot](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_robot)
* [irob_motion](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_motion)
* [irob_vision_support](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_vision_support)
* [irob_subtask_logic](https://github.com/ABC-iRobotics/irob-saf/tree/master/irob_subtask_logic)


## Build

The framework is tested on Ubuntu 20.04 LTS with ROS Noetic, so this platform is encouraged. The building process can be performed using catkin build tools for ROS (so use `catkin build`, but you should never `catkin_make`) by the following steps.

### Install ROS Noetic

Type:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-noetic-desktop-full

Initialize rosdep:

    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update

Setup environment:

    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

### Install dependencies

#### Install dependencies for building packages

    sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools 


#### Install Eigen

This sowfware is using the [Eigen C++ template library](http://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix and vector classes and algorithms. Install Eigen as follows:

    sudo apt install libeigen3-dev

Now the Eigen headers should be in your /usr/include/eigen3 directory. ROS packages using the Eigen library have to list the /usr/include/eigen3 in the include_directories in the CMakelist.txt file of the package (already done for irob-saf packages), for example:

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(include ${catkin_INCLUDE_DIRS} 
          /usr/include/eigen3
    )
 

### Build dVRK
 
The library can be used stand-alone, but it was developed to use with the [da Vinci Reserach Kit 2.1.x](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki), icluding the [cisst-saw](https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros) and the [dvrk-ros](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#dvrk-ros) packages. To install these packages, use do the following steps (see the install `cisst-saw` and `dvrk-ros` at [https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-workspace-clone-and-build](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild#catkin-workspace-clone-and-build)):


    mkdir ~/catkin_ws                  # create the catkin workspace
    cd ~/catkin_ws                     # go in the workspace
    catkin init                        # create files for catkin build tool
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
    mkdir src
    cd src                             # go in source directory to pull code
    vcs import --recursive --workers 1 --input https://raw.githubusercontent.com/jhu-saw/vcs/main/ros1-dvrk-2.1.0.vcs    
    cd ~/catkin_ws
    catkin build --summary             # ... and finally compile everything



### Build irob-saf

The `irob-saf` repository can be pulled using `rosinstall` or `git clone`. For the plain usage of the framework, `rosinstall` is an easy and convenient way; if you plan to modify the source code, the usage of `git clone` is the better choice. Regardless of the method of pulling the repo, it can be built using `catkin build`.

### A) Get and build irob-saf using `rosinstall` and `catkin build`

If you installed dVRK by the `catkin build` and `rosinstall` method, you should already have a catkin_ws directory with `wstool` set-up properly. A `rosinstall` file is also provided to `irob-saf`, making the build process easier. So, if you used the `catkin build` and `rosinstall` to build dVRK, `irob-saf` can be built using the following commands:

    source ~/catkin_ws/devel/setup.bash
    cd ~/catkin_ws/src
    wstool merge https://raw.githubusercontent.com/ABC-iRobotics/irob-saf/master/irob-saf/irob_saf.rosinstall   # or replace "master" with the corresponding branch
    wstool up
    cd ~/catkin_ws
    catkin build irob-saf
    
Setup environment:

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc


### B) Get and build irob-saf using `calkin build` and `git clone` 

The other option to build `irob-saf` is to simply clone the repository into your workspace. If your have installed dVRK, you should already have a catkin_ws directory set-up properly. Elsehow, do the following:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    
If you alread have a workspace, just download the sources:

    cd ~/catkin_ws/src
    git clone https://github.com/ABC-iRobotics/irob-saf
    
And build using `catkin build`:

    cd ~/catkin_ws
    source devel/setup.bash
    catkin build irob-saf

Setup environment:

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    

    
*And nothing left but to say congratulations, you have succesfully installed the `irob-saf` framework!*  

The package `irob_vision_support` uses some additional dependencies. If some dependecies are missing during build or run, please see the `README` in the package `irob_vision_support`.


## Usage

Autonomous surgical subtasks can be assembled from the ROS nodes of the framework based on the example in the figure below. To build a custom solution, you should first look at the packages of the framework.

### Grasp in simulator

An example using the dVRK PSM simulation and a dummy target can be launched easily however. First, start the simulator:

    roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/$(whoami)/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json
    
If the simulation is started, press *Home*.  
Start a dummy target:

    roslaunch irob_vision_support dummy_target.launch 
    
This command will start a node that publishes a marker position. To visualize this marker, in the RViz window's left panel press *Add*, chose *By topic*, and find */saf/vision/dummy_target_marker*. Now you should see a small green sphere in front of the PSM.  
Start a dummy vision node, that will simply republish the position of this marker:

    roslaunch irob_vision_support dummy_vision.launch 
    
Start the high-level robot control node for the PSM by typing:

    roslaunch irob_robot dvrk_server.launch arm_typ:=PSM1 camera_registration_file:=registration_identity.yaml instrument_info_file:=large_needle_driver.yaml
    
Start a surgeme server:

    roslaunch irob_motion surgeme_server.launch

And finally, start the subtask-level logic node:

    roslaunch irob_subtask_logic dummy_grasp.launch 
    
After the last node was launched, you should see how the instrument grasps the green sphere.  
  

![alt tag](docs/irob-autosurg-blockdiagram.png)

### Unilateral peg transfer

The follwoing example shows how to launch the example *Unilateral peg transfer* on the physical robot and DVRK.  


First, wire and power up the DVRK controller for PSM1. Disconnect, and then reconnect Firewire (physically). Then open a terminal and type:

    roscore
    
In a separate terminal:

    cd ~/catkin_ws/share
    rosrun dvrk_robot dvrk_console_json -j <CALIBRATION_FOLDER>/console-PSM1.json 

Home the robot using the DVRK console.  
    
In separate terminals: 
  
    roslaunch irob_vision_support peg_transfer_perception.launch 
 
    roslaunch irob_robot dvrk_server.launch arm_typ:=PSM1 camera_registration_file:=registration_psm1.yaml instrument_info_file:=large_needle_driver.yaml
    
    roslaunch irob_motion surgeme_server.launch

    roslaunch irob_subtask_logic peg_transfer_unilateral.launch mode:=execution
    


### Bilateral peg transfer

The follwoing example shows how to launch the example *Bilateral peg transfer* on the physical robot and DVRK.  


First, wire and power up the DVRK controller for PSM1 and PSM2. Disconnect, and then reconnect Firewire (physically). Then open a terminal and type:

    roscore
    
In a separate terminal:

    cd ~/catkin_ws/share
    rosrun dvrk_robot dvrk_console_json -j <CALIBRATION_FOLDER>/console-PSM1-PSM2.json

Home the robot using the DVRK console.  
    
In separate terminals: 
  
    roslaunch irob_vision_support peg_transfer_perception.launch 
 
    roslaunch irob_robot dvrk_server.launch arm_typ:=PSM1 camera_registration_file:=registration_psm1.yaml instrument_info_file:=large_needle_driver.yaml

    roslaunch irob_robot dvrk_server.launch arm_typ:=PSM2 camera_registration_file:=registration_psm2.yaml instrument_info_file:=large_needle_driver.yaml arm_name:=arm_2
    
    roslaunch irob_motion surgeme_server.launch arm_name:=arm_1

    roslaunch irob_motion surgeme_server.launch arm_name:=arm_2

    roslaunch irob_subtask_logic peg_transfer_bilateral.launch
    

## Contact

The `irob-saf` package is built and maintained in the Antal Bejczy Center for Intelligent Robotics, see our [homepage](http://irob.uni-obuda.hu/?q=en). 

If you have any questions or comments, feel free to contact us at [saf@irob.uni-obuda.hu](mailto:saf@irob.uni-obuda.hu).


## Acknowledgement
We acknowledge the financial support of this work by the Hungarian State and the European Union under the  EFOP-3.6.1-16-2016-00010 project.

