# This file contains definitions of variables needed for the installation of an OpenVPN gateway
# with static IP address, and a single VXLAN connection to a second cloud computer.
#
# Gateway private cloud IP address
ROSVPN_GATEWAY_CLOUD_IP=172.31.1.11
# ROS2 local network IP address of gateway bridge (with /CIDR)
ROSVPN_GATEWAY_LOCAL_IP=192.168.0.130/24
# Disable ROS2 install in gateway, comment out to allow
ROSVPN_SKIP_ROS2=true
# Disable Docker install in gateway, comment out to allow
ROSVPN_SKIP_DOCKER=true
# Optional DNS name of gateway public IP address (will use acual IP if undefined)
#OPENVPN_URL=vpn.rosdabbler.com
#
# These variables with VXLAN1 should be commented out if no VXLAN1 is wanted
# Should we define a VXLAN to connect to a second system?
ROSVPN_VXLAN1=true
# Second cloud instance VX1 private cloud IP address
ROSVPN_VXLAN1_CLOUD_IP=172.31.1.12
# ROS2 local network IP address of gateway vxlan1 endpoint
ROSVPN_VXLAN1_GATEWAY_LOCAL_IP=192.168.0.145
# ROS2 local network IP address of VX1 (in   /30 network with ROSVPN_GATEWAY_LOCAL_IP)
ROSVPN_VXLAN1_ENDPOINT_LOCAL_IP=192.168.0.146
