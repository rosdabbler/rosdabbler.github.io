# usage:
# source ain5.install
#
# Install script starting for aws openvpn test. This script requires that variables be defined
# before running, using:
#
# source ain5-configure.sh
#
# of they will be asked for when run,
#

## Optional variables
#
# Disable ROS2 install in gateway, comment out to allow
#ROSVPN_SKIP_ROS2=true
# Disable Docker install in gateway, comment out to allow
#ROSVPN_SKIP_DOCKER=true
# Optional DNS name of gateway public IP address (will use acual IP if undefined)
#OPENVPN_URL=vpn.rosdabbler.com

# Required variables, request values if undefined
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

get_or_set ROSVPN_GATEWAY_VPN_IP 192.168.0.130/24
get_or_set ROSVPN_GATEWAY_CLOUD_IP 172.31.1.11
get_or_set ROSVPN_SECOND_CLOUD_IP 172.31.1.12

sudo apt-get update && sudo apt-get upgrade -y

# For AWS installs, make sure we are not wasting memory on graphics
sudo systemctl isolate multi-user
sudo systemctl set-default multi-user

# prevent keyboard config request to user
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

# Stuff we need for install, or to fix if interrupted
sudo apt install -y nano git wget curl less

# netplan configuration to create the bridge that ROS2 will use
cat << EOF | sudo tee /etc/netplan/90-rosbridge.yaml
network:
  version: 2

  tunnels:
    gre1:
      renderer: networkd
      mode: gretap
      remote: $ROSVPN_SECOND_CLOUD_IP
      local: $ROSVPN_GATEWAY_CLOUD_IP

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [gre1]
      parameters:
        stp: false
      addresses: [$ROSVPN_GATEWAY_VPN_IP]
      dhcp4: false
      dhcp6: false
EOF

# go ahead and create the bridge
sudo netplan apply

##  Docker install see https://docs.docker.com/engine/install/ubuntu/

# When Docker is running, it sets the default acceptance of the FORWARD chain to DENY,
# and loads the **br_netfilter** kernel module so that layer 2 forwards on the bridge
# go through the iptables FORWARD chain. The net result is that unless we explictly allow
# rosbridge forwards in iptables, they will fail. The following section will fix that.
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent
sudo iptables -F DOCKER-USER 2> /dev/null || sudo iptables -N DOCKER-USER
sudo iptables -A DOCKER-USER -i rosbridge -o rosbridge -j ACCEPT
sudo iptables -A DOCKER-USER -j RETURN
sudo iptables-save -f /etc/iptables/rules.v4

if [[ ! "$ROSVPN_SKIP_DOCKER" = true ]]; then
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

if [[ ! "$ROSVPN_SKIP_ROS2" = true ]]; then
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

# Download kylemanna/openvpn docker install which has some nice scripts we will be using
if [ ! -d docker-openvpn ]; then
    git clone https://github.com/kylemanna/docker-openvpn.git
    git -C docker-openvpn/ checkout 1228577
fi

# Packages needed to install from kylemanna/openvpn Dockerfile
sudo apt-get install -y \
  openvpn easy-rsa

# Other commands from kylemanna/openvpn
sudo cp -r docker-openvpn/bin/* /usr/local/bin/
sudo chmod a+x /usr/local/bin/*
sudo ln -s /usr/share/easy-rsa/easyrsa /usr/local/bin

# Add openvpn environment variables, and tell Cyclone DDS to use rosbridge
if ! grep -q -F "OPENVPN" /etc/environment; then
cat <<EOF | sudo tee -a /etc/environment
OPENVPN=/etc/openvpn
EASYRSA=/usr/share/easy-rsa
EASYRSA_CRL_DAYS=3650
EASYRSA_PKI=/etc/openvpn/pki
CYCLONEDDS_URI='<General><NetworkInterfaceAddress>rosbridge</></>'
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
  lsof \
  avahi-daemon

# enable forwarding
echo "net.ipv4.ip_forward = 1" | sudo tee /etc/sysctl.d/99-ipforward.conf
sudo sysctl -w net.ipv4.ip_forward=1

# hostname
sudo hostnamectl set-hostname rosvpn-gw
sudo systemctl restart avahi-daemon

# set the default OPEN_URL with the current IP address
if [[ -z "$OPENVPN_URL" ]]; then
    OPENVPN_URL=$(curl icanhazip.com)
    echo "OPENVPN_URL is $OPENVPN_URL"
fi

# OpenVPN configuration using the Docker scripts, adapted for local use.
#
# You can also look at 
# to see the meaning of various setup options.
#
sudo rm /etc/openvpn/ovpn_env.sh
UP_COMMAND="/bin/bash -c 'ip link set master rosbridge dev tap0 && ip link set up tap0'"
sudo ovpn_genconfig -u udp://$OPENVPN_URL -t -d -D \
  -e "ifconfig-noexec" \
  -e "script-security 2" \
  -e "up \"${UP_COMMAND}\"" \
  -E "ifconfig-noexec" \
  -E "script-security 2" \
  -E "up \"${UP_COMMAND}\""
