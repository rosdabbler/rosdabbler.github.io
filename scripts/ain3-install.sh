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
sudo apt install -y nano git wget

# Get values of config variables
if [ -z "$ROVPN_SUBNET" ]; then
  read -p "Enter ROVPN_SUBNET: " ROVPN_SUBNET
fi

if [ -z "$ROVPN_CLOUD_GATEWAY"]; then
  read -p "Enter ROVPN_CLOUD_GATEWAY" ROVPN_CLOUD_GATEWAY
fi

##  Docker install see https://docs.docker.com/engine/install/ubuntu/

# Docker prelims
sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# set the default OPEN_URL with the current IP address
if [-z "$OPENVPN_URL" ]; then
    OPENVPN_URL=$(curl icanhazip.com)
    echo "OPENVPN_URL is $OPENVPN_URL"
fi

# Add Docker's official GPG key
 curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Setup Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker engine
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker ubuntu

## Install ROS2

# Ref: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-galactic-desktop

# Download kylemanna/openvpn docker install which has some nice scripts we will be using
if [ ! -d docker-openvpn ]; then
    git clone https://github.com/kylemanna/docker-openvpn.git
    git -C docker-openvpn/ checkout 1228577
fi

# Source ROS2 by default
ROS2_SOURCE="source /opt/ros/galactic/setup.bash"
if grep -q -F "$ROS2_SOURCE" ~/.bashrc; then
  echo "ROS2 already sourced by default"
else
  echo "$ROS2_SOURCE" >> ~/.bashrc
fi

# Packages needed to install from kylemanna/openvpn Dockerfile
sudo apt-get install -y \
  openvpn easy-rsa

# Other commands from openvpn Dockerfile
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

# Additional packages which to help configure or debug networking
sudo apt-get install -y \
  bridge-utils \
  dnsutils \
  iputils-ping \
  net-tools \
  lsof

# enable forwarding
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-ipforward.conf
sudo sysctl -w net.ipv4.ip_forward=1

# hostname
sudo hostnamectl set-hostname rosovpn

# create the up.sh file that will run when link comes up
sudo mkdir -p /etc/openvpn
cat <<END | sudo tee /etc/openvpn/up.sh > /dev/null
#!/bin/bash
#
# This is run when device comes up
#
mkdir -p /var/log/rovpn
env >> /var/log/rovpn/up.log
ip link set master rosbridge dev $dev
ip link set rosbridge up
ip link set $dev up
END

# create the file to persist the bridge if Docker not started
cat <<END | sudo tee /etc/netplan/90-rosbridge.yaml > /dev/null
network:
  version: 2
  bridges:
    rosbridge:
      addresses: [$ROVPN_SUBNET]
      mtu: 1500
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
END

# apply to create the bridge
sudo netplan apply

# create the Docker version of the bridge
sudo docker network create --driver=bridge \
  --subnet=$ROVPN_SUBNET \
  --ip-range=$ROVPN_CLOUD_GATEWAY_DOCKER_IPS \
  --gateway=$ROVPN_CLOUD_GATEWAY \
  --opt com.docker.network.bridge.name=rosbridge \
  rosbridge
# go back to the previous directory
cd -
