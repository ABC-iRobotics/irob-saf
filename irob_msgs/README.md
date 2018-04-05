# irob_msgs
Messages, services and actions for irob-autosurg

## Matlab custom message generation

Matlab requires aditional build steps to be able to access this specific mesage types. Do the following steps:

- Install add-on "Robotics System Toolbox Interface for ROS Custom Messages" (use sudo matlab, elsehow nothing will happen)
- Relaunch Matlab in non-sudo mode
- Type in Matlab console:

        folderpath = fullfile('catkin_ws','src', 'irob-autosurg')
        rosgenmsg(folderpath)
        
- Follow the instructions suggested by rosgenmsg (savepath works only in sudo mode)

For a more detailed guide see https://www.mathworks.com/help/robotics/ug/create-custom-messages-from-ros-package.html
