 #!/bin/bash
 gnome-terminal -x bash -c "roscore; sleep 10000"
sleep 2
 gnome-terminal -x bash -c "rosrun robair_demo kb_control.py; sleep 10000"
 gnome-terminal -x bash -c "rosrun robair_demo motion_control_node.py; sleep 10000"
 gnome-terminal -x bash -c "rosrun robair_demo arduino_sensors.py; sleep 10000"
