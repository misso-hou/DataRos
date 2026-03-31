#!/bin/bash

# 参考网址
# https://alidocs.dingtalk.com/i/nodes/NDoBb60VLQeexG50CPmmX0p9JlemrZQ3?utm_source=im&cid=567237149%3A1015931361&utm_scene=person_space&iframeQuery=utm_medium%3Dim_card%26utm_source%3Dim&utm_medium=im_card&corpId=dingd62fc09b6aa94493ee0f45d8e4f7c288

# 设置变量，提供默认值
VEHICLE_NAME="${VEHICLE_NAME:-pde-l4e-b0003}"
ROS_IP_ADDRESS="${ROS_IP_ADDRESS:-192.168.46.100}"

echo "Configuring systemd services for ROS integration..."
cd "/opt/plusai/config/systemd/${VEHICLE_NAME}/"
sed -i \
  -e 's/IPC_PUBSUB_PUBLISHER_MODES="shm_bus"/IPC_PUBSUB_PUBLISHER_MODES="ros,shm_bus"/g' \
  -e 's/ROS_MASTER_URI="http:\/\/localhost:11311"/ROS_MASTER_URI="http:\/\/0.0.0.0:11311"/g' \
  -e "s/ROS_IP=\"192.168.11.100\"/ROS_IP=\"${ROS_IP_ADDRESS}\"/g" \
  -e "s/ROS_IP=192.168.11.100/ROS_IP=${ROS_IP_ADDRESS}/g" * 

echo "update rosmaster config"
sed -i \
  -e 's/^export ROS_MASTER_URI=.*$/export ROS_MASTER_URI=http:\/\/0.0.0.0:11311/' \
  -e "s/^export ROS_IP=.*$/export ROS_IP=${ROS_IP_ADDRESS}/" \
  /opt/plusai/launch/l4e-common/start-rosmaster.sh

echo "update setup script"
sed -i '$a\export ROS_IP='"${ROS_IP_ADDRESS}"'\nexport ROS_MASTER_URI="http://0.0.0.0:11311"' "/opt/plusai/launch/${VEHICLE_NAME}/setup.sh"

echo "source vehicle setup script"
. "/opt/plusai/launch/${VEHICLE_NAME}/setup.sh"

echo "do init"
hamlaunch init
