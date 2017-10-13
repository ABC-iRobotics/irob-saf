# irob_robot_interface
Generic interface to surgical robots. Currently supports only the dVRK, other platforms will be hopefully added in future releases.

# Useful informations about dVRK

Timing:
  - dVRK bridge: ~1 kHz
  - Time for one command in ROS: 80 - 160 us
  
PSM workspace:
  - joints:
  	- #0 outer_yaw (rad): 			(-1.3, 1.3)
  	- #1 outer_pitch (rad): 		(-0.8, 0.8)
  	- #2 outer_insertion (m): 		(0.0, 0.24) enable cartesian: > 0.05
  	- #3 outer_roll (rad): 			(-2.8, 2.8)
  	- #4 outer_wrist_pitch (rad): 	(-1.3, 1.3)
  	- #5 outer_wrist_yaw (rad): 	(-1.3, 1.3)
  	- #6 jaw (rad): 				(0.0, 1.3)
  - cartesian  (in meters):
    - origin: cannula center point
      - x: (-0.215, 0.215)
      - y: (-0.175, 0.160)
      - z: (-0.040, -0.222)
  - HW and simulator coordinate systems are compatible and matching
  
      
![alt tag](../docs/PSM_coordinates.png)
