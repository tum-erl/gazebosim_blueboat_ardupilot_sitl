

sudo apt update
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras

sudo apt install ros-${ROS_DISTRO}-mavros-msgs

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
(ODER:
    sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-gravity egm2008
sudo geographiclib-get-magnetic wmm2020)


ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://@192.168.2.2:14600 \
  -p system_id:=255 -p component_id:=190 \
  -p tgt_system:=1 -p tgt_component:=1


ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1600, 0, 1600, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"

ros2 topic pub -r 10 /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"


ros2 topic echo /mavros/gpsstatus/gps1/raw



ros2 topic pub /asv/stop std_msgs/msg/Bool "data: true"
