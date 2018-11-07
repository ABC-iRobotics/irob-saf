# irob_connect

Package for publishing the joint states of the DVRK on a websocket.

To enable execution of Python node, type

    chmod +x <path>/catkin_ws/src/irob_saf/irob_connection/scripts/dvrk_rosbridge_upstream.py

To launch publisher, type

    roslaunch irob_connect dvrk_rosbridge_upstream.launch host:=<IP_address_of_host>
