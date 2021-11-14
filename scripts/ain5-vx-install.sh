# usage:
# source ain5-vx-install.sh
#
# Install script starting for aws openvpn test. This script requires that variables be defined
# before running, or they will be asked for when the script is sourced.
#
# Required variables:
#
# The IP address that the second instance should have in the IP subnet used by the robot.
# In address/size format
#ROSVPN_SECOND_VPN_IP=192.168.0.131/24
# The IP address of the gateway instance in the cloud network.
#ROSVPN_GATEWAY_CLOUD_IP=172.31.1.11
# The IP address of the second computer in the cloud network.
#ROSVPN_SECOND_CLOUD_IP=172.31.1.12
#
## Optional variables
#
# Disable ROS2 install
#ROSVPN_SKIP_ROS2=true
# Disable Docker install
#ROSVPN_SKIP_DOCKER=true

# Required variables function, request values if undefined
get_or_set() {
  local var=$1
  local def=$2
  local result=""
  if [[ -z "${!var}" ]]; then
    read -p "Enter ${var} [${def}] > " result
    eval $var="${result:-$def}"
    echo "$var = ${!var}"
  fi
}

get_or_set ROSVPN_SECOND_VPN_IP 192.168.0.131/24
get_or_set ROSVPN_GATEWAY_CLOUD_IP 172.31.1.11
get_or_set ROSVPN_SECOND_CLOUD_IP 172.31.1.12

sudo apt-get update && sudo apt-get upgrade -y

# For small AWS installs, make sure we are not wasting memory on graphics
sudo systemctl isolate multi-user
sudo systemctl set-default multi-user

# prevent keyboard config request to user
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

# Stuff we need for install, or to fix if interrupted
sudo apt install -y nano git wget curl less

# netplan configuration to create a tunnel to the gateway
cat << EOF | sudo tee /etc/netplan/90-gre1.yaml
network:
  version: 2

  tunnels:
    gre1:
      renderer: networkd
      mode: gretap
      remote: $ROSVPN_GATEWAY_CLOUD_IP
      local: $ROSVPN_SECOND_CLOUD_IP
      addresses: [$ROSVPN_SECOND_VPN_IP]
EOF

# go ahead and create the bridge
sudo netplan apply

# Tell Cyclone DDS to use gre1 tunnel with ROS2
if ! grep -q -F "CYCLONEDDS_URI" /etc/environment; then
cat <<EOF | sudo tee -a /etc/environment
CYCLONEDDS_URI='<General><NetworkInterfaceAddress>gre1</></>'
EOF
fi

# also set these locally
source /etc/environment

##  Docker install see https://docs.docker.com/engine/install/ubuntu/
if [[ ! "$ROSVPN_SKIP_DOCKER" == true ]]; then
    # Docker prelims
    sudo apt-get install \
        ca-certificates \
        gnupg \
        lsb-release

    # Add Docker's official GPG key
    if [[ ! -f /usr/share/keyrings/docker-archive-keyring.gpg ]]; then
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    fi

    # Setup Docker repository
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    # Install Docker engine
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    sudo usermod -aG docker ubuntu
fi

## Install ROS2
if [[ ! "$ROSVPN_SKIP_ROS2" == true ]]; then
    # Ref: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt install -y ros-galactic-desktop

    # Source ROS2 by default
    ROS2_SOURCE="source /opt/ros/galactic/setup.bash"
    if grep -q -F "$ROS2_SOURCE" ~/.bashrc; then
        echo "ROS2 already sourced by default"
    else
        echo "$ROS2_SOURCE" >> ~/.bashrc
    fi
    source /opt/ros/galactic/setup.bash
fi

# Optional packages which may help configure or debug networking
sudo apt-get install -y \
  dnsutils \
  iputils-ping \
  net-tools \
  lsof \
  avahi-daemon

# hostname
sudo hostnamectl set-hostname rosvpn-second
sudo systemctl restart avahi-daemon
