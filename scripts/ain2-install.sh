#!/bin/bash
#
# Install script starting for aws openvpn test
#
sudo apt-get update && sudo apt-get upgrade -y

# For AWS installs, make sure we are not wasting memory on graphics
sudo systemctl isolate multi-user
sudo systemctl set-default multi-user

# prevent keyboard config request to user
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

# Stuff we need to do install, or to fix if interrupted
sudo apt install -y nano git wget curl less

# Install ROS2
# Ref: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-galactic-desktop

# Source ROS2 by default
ROS2_SOURCE="source /opt/ros/galactic/setup.bash"
if grep -q -F "$ROS2_SOURCE" ~/.bashrc; then
  echo "ROS2 already sourced by default"
else
  echo "$ROS2_SOURCE" >> ~/.bashrc
fi
source $ROS2_SOURCE

# Download kylemanna/openvpn docker install which has some nice scripts we will be using
git clone https://github.com/kylemanna/docker-openvpn.git
git -C docker-openvpn/ checkout 1228577

# Packages from kylemanna/openvpn Dockerfile
sudo apt-get install -y \
  openvpn easy-rsa

# Other commands from kylemanna/openvpn
sudo cp -r docker-openvpn/bin/* /usr/local/bin/
sudo chmod a+x /usr/local/bin/*
sudo ln -s /usr/share/easy-rsa/easyrsa /usr/local/bin

# Add openvpn environment variables
if ! grep -q -F "OPENVPN" /etc/environment; then
cat <<EOF | sudo tee -a /etc/environment
OPENVPN=/etc/openvpn
EASYRSA=/usr/share/easy-rsa
EASYRSA_CRL_DAYS=3650
EASYRSA_PKI=/etc/openvpn/pki
EOF
fi

# also set these locally
source /etc/environment

# Additional packages which may help configure or debug networking
sudo apt-get install -y \
  bridge-utils \
  dnsutils \
  iputils-ping \
  net-tools \
  lsof

# enable forwarding
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-ipforward.conf
sudo sysctl -w net.ipv4.ip_forward=1

# set the default OPEN_URL with the current IP address
if [[ -z "$OPENVPN_URL" ]]; then
    OPENVPN_URL=$(curl icanhazip.com)
    echo "OPENVPN_URL is $OPENVPN_URL"
fi
