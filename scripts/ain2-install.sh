#!/bin/bash
#
# Install script starting for aws openvpn test
#
sudo apt-get update && sudo apt-get upgrade -y

# For AWS installs, make sure we are not wasting memory on graphics
sudo systemctl isolate multi-user
sudo systemctl set-default multi-user

mkdir /tmp/install
cd /tmp/install

# prevent keyboard config request to user
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

# Stuff we need to do install, or to fix if interrupted
sudo apt install -y curl nano git wget
# sudo apt install -y gnupg lsb-release

# Install ROS2
# Ref: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-galactic-desktop

# ROS1 install
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install -y ros-noetic-desktop

# Download kylemanna/openvpn docker install which has some nice scripts we will be using
if [ ! -d docker-openvpn ]; then
    git clone https://github.com/kylemanna/docker-openvpn.git
    git -C docker-openvpn/ checkout 1228577
fi

# Source ROS2, also ROS1 but commented out by default
ROS2_SOURCE="source /opt/ros/galactic/setup.bash"
if grep -q -F "$ROS2_SOURCE" ~/.bashrc; then
  echo "ROS2 already sourced by default"
else
  echo "$ROS2_SOURCE" >> ~/.bashrc
fi

ROS1_SOURCE="source /opt/ros/noetic/setup.bash"
if grep -q -F "#$ROS1_SOURCE" ~/.bashrc; then
  echo "ROS1 already sourced by default"
else
  echo "#$ROS1_SOURCE" >> ~/.bashrc
fi

# Packages from kylemanna/openvpn Dockerfile
sudo apt-get install -y \
  openvpn easy-rsa

sudo cp -r docker-openvpn/bin/* /usr/local/bin/
sudo chmod a+x /usr/local/bin/*

# Other commands from openvpn Dockerfile
sudo ln -s /usr/share/easy-rsa/easyrsa /usr/local/bin

# Add openvpn environment variables

if ! grep -q -F "OPENVPN" /etc/environment; then
OPENVPN_ENV=$(cat <<'EOF'
OPENVPN=/etc/openvpn
EASYRSA=/usr/share/easy-rsa
EASYRSA_CRL_DAYS=3650
EASYRSA_PKI=/etc/openvpn/pki
EOF
)
echo "$OPENVPN_ENV" | sudo tee -a /etc/environment
fi

# Additional packages whch to help configure or debug networking
sudo apt-get install -y \
  bridge-utils \
  dnsutils \
  iputils-ping

# enable forwarding
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-ipforward.conf
sudo sysctl -w net.ipv4.ip_forward=1

# stuff I want for AWS installs
sudo hostnamectl set-hostname openvpn

# the previous directory
cd -
