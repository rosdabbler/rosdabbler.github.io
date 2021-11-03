# This script contains variables that may be defined for an
# OpenVPN gateway with Docker and ROS2 on a virtual machine. All
# variables have defaults that will work. The sample definitions
# shown are not defaults but examples if set.

# The IP range that will be used by ROS2 and Docker. Leave blank
# to use DHCP from the remote gateway
#ROSVPN_SUBNET=10.231.160.0/20
#
# The IP address that will be used by the OpenVPN connection.
# Leave blank if using DHCP.
#ROSVPN_CLOUD_GATEWAY=10.231.161.1
#
# The URL that will be used to contact the OpenVPN server. If
# blank, the public IP address of the OpenVPN server will be
# detected and used.
#
#OPENVPN_URL=vpn.rosdabbler.com
#
# Uncomment to skip installing ROS2
#ROSVPN_SKIP_ROS2="true"
#
# Uncomment to skip installing Docker
#ROSVPN_SKIP_DOCKER="true"