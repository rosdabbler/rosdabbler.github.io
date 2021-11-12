# This file contains definitions of variables needed for the installation of an OpenVPN gateway
# with static IP address, and a single VXLAN connection to a second cloud computer. These
# two instances are called GATEWAY and SECOND. Addresses that are used by the robot network
# are called VPN IPs. Private IPs used in the cloud network are called CLOUD IPs. 
#
# ROS2 local network IP address of gateway bridge (with /CIDR)
ROSVPN_GATEWAY_VPN_IP=192.168.0.130/24
# Disable ROS2 install in gateway, comment out to allow
ROSVPN_SKIP_ROS2=true
# Disable Docker install in gateway, comment out to allow
ROSVPN_SKIP_DOCKER=true
# Optional DNS name of gateway public IP address (will use acual IP if undefined)
#OPENVPN_URL=vpn.rosdabbler.com
#
# Should we define a VXLAN to connect to a second system? true or undefined
ROSVPN_SECOND=true
# These variables below are only needed if ROSVPN_SECOND=true
# Gateway private cloud IP address (only needed in SECOND config)
ROSVPN_GATEWAY_CLOUD_IP=172.31.1.11
# Second cloud instance private cloud IP address
ROSVPN_SECOND_CLOUD_IP=172.31.1.12
# ROS2 robot network IP address of gateway vxlan1 endpoint
ROSVPN_VXLAN1_GATEWAY_VPN_IP=192.168.0.145
# ROS2 robot network IP address of second system
# (in /30 network with ROSVPN_VXLAN1_GATEWAY_VPN_IP)
ROSVPN_VXLAN1_SECOND_VPN_IP=192.168.0.146
