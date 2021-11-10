---
layout: single
title:  "Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway"
tags: AWS networking OpenVPN
excerpt: "This is a complete installation of the ROS2-compatible OpenVPN Cloud Gateway"
permalink: adventures-in-ros2-networking-4
---
## Introduction

In other posts in this series, I've gone into various issues associated with OpenVPN, ROS2, and Docker. But in the end, you just need the **How To** guide, that gives what you really need to make a simple installation work. This is it.

The basic setup is there are three processors, each that might run ROS2 nodes on either the host, or on Docker containers: a robot, a desktop computer serving as an OpenVPN gateway, and a Cloud Gateway.

![Robot and OpenVPN Gateways](/assets/images/OpenVpnGateways4.png)

## Overall Plan

The intent of this installation plan is to be as minimally intrusive on an existing setup as possible. To do that, the configuration will make the following choices:
- Any Docker containers use host networking, that is the option ```--net=host --pid=host```
- The Desktop Gateway and the Cloud Gateway get their IP addresses from a DHCP router in the local network (that is, the same network as the Desktop and Robot). This is not essential, later I'll give an example with static IP addresses for both.
- You have complete control of the Desktop Gateway and the Cloud Gateway.
- No changes are needed on the Robot, assuming it can successfully communicate with ROS2 nodes running on the Desktop prior to this setup.
- The Desktop is running Ubuntu 20.04. I have not tested this on Windows, macOS, or other versions of Linux -- though the principles should be similar.
- ROS2 is using Cyclone DDS, which is the default provider for ROS2 Galactic.

The OpenVPN tunnel between the Desktop Gateway and the Cloud Gateway will be configured as a Layer 2 (Linux link tap0) tunnel. This is necessary for the ROS2 DDS discovery process to find other nodes.

Both the Desktop Gateway and the Cloud Gateway need to be configured so that their network connection goes through a Linux bridge rather than directly through an ethernet adapter. The bridge on the Desktop Gateway will not interfere with normal operation of the Desktop when OpenVPN is uninstalled or disconnected, though it does prevent using the GUI Network Settings to configure internet access.

The Layer 2 bridge serves the same function as a Layer 3 router, that is deciding how to route packets between various processors. When an IP packet arrives from the robot over the ethernet connection, the Desktop Gateway has to decide if it should send it over the tap0 tunnel. The bridge is what allows it to make that decision.

On the Cloud Gateway, we are also installing a bridge here. That is not strictly necessary for the current configuration, but it will be needed when, paralleling the issues on the local network with the robot, we want to run additional cloud processors on the same cloud virtual network.

For the Cloud Gateway OpenVPN setup, we will be using scripts from a [popular Docker image](https://github.com/kylemanna/docker-openvpn "OpenVPN server in a Docker container complete with an EasyRSA PKI CA ") for setting up an OpenVPN gateway - but we will only be using the scripts from that, not running it under Docker. Two reasons for this: first, it gets OpenVPN working quickly but without dealing with extra issues introduced by Docker, and second it makes it easier to switch to using the Docker container eventually.

In [part 2](/adventures-in-ros2-networking-2 "Adventures in ROS2 Networking 2 - OpenVPN gateway ") of this series, we installed am OpenVPN gateway that we then tested in [part 3](/adventures-in-ros2-networking-3 "Adventures in ROS2 Networking 3 - ROS2 over OpenVPN Gateway"). At this point, if you are trying to follow through these demos, it is time to abandon that installation, and start with a fresh cloud virtual machine. The starting point of this machine though is the same as before.

To install and test OpenVPN and ROS2 on a virtual machine to serve as the Cloud Gateway, you'll need a virtual machine with:
- Public, permanent IP address
- Firewall setup to accept:
    - SSH (TCP port 22)
    - OpenVPN (UDP port 1194)
    - Ping (ICMP Echo Request)
- 0.5 GiB memory, 1 CPU, 8 GB drive is sufficient, though bigger might be better particularly if you are really going to run ROS2 nodes there.
- User with sudo privileges with ssh access.
- Minimal Ubuntu 20.04 installation.

For the Cloud Gateway, we'll be using an installation script available on this site [here](https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain4-install.sh). Look that over to make sure there's nothing going on that you are uncomfortable with. But before we do that installation, there is some configuration to do on the local Desktop Gateway.

## Desktop Gateway Setup

We need to:
- create the bridge, and add our ethernet adapter to it
- install OpenVPN
- configure ROS2 DDS to use the bridge adapter for communication
- add **iptables** rules to allow bridge routing when Docker is enabled

### Create the Bridge
We'll use [Netplan](https://netplan.io/reference/) to configure network connections, which will persist these changes between reboots. This netplan file creates a bridge named **rosbridge**, configures it to use dhcp, and adds any ethernet adapters as links under the bridge. **When you add this file to your computer, you are overriding the default use of NetworkManager and the related GUI to configure your network connection. Any configuration (such as a static local IP address, or alternate DNS servers) needs to be done in this file to the rosbridge definition.**
```yaml
network:
  version: 2
  ethernets:
    estar:
      match:
        name: "e*"

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [estar]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
Add this file as /etc/netplan/90-rosbridge.yaml, either manually or with this script:
```bash
sudo curl https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/90-rosbridge.yaml -o /etc/netplan/90-rosbridge.yaml
```
Once this file is added, on reboot the bridge will be created and used. To apply this immediately, go ahead and execute:
```bash
sudo netplan apply
```
Test your internet:
```bash
kent@ubutower:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eno1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UP group default qlen 1000
    link/ether d0:17:c2:99:00:a4 brd ff:ff:ff:ff:ff:ff
    altname enp0s25
5: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether d0:17:c2:99:00:a4 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.55/24 brd 192.168.0.255 scope global dynamic noprefixroute rosbridge
       valid_lft 7199sec preferred_lft 7199sec
    inet6 fe80::d4d5:8cff:fe1a:b183/64 scope link 
       valid_lft forever preferred_lft forever
kent@ubutower:~$ ping 8.8.8.8
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=116 time=2.80 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=116 time=2.82 ms
```
Under ```ip a```, what your are looking for is:
- a bridge **rosbridge** is created, and it has an ip address on the local network
- the ethernet adapter has **master rosbridge** in its properties.
If something goes wrong, delete /etc/netplan/90-rosbridge.yaml and ```sudo netplan apply``` (or reboot) to recover.

Because of [this issue](/notes/2021/11/netplan-bridge-add-fails-eno1/ "Netplan Apply Failure with tap0 and eno1") sometimes the bridge will not properly come up even with `sudo netplan apply`. Reboot seems to work reliably, or take down your ethernet adapter first. That is, assuming your ethernet adapter name is **eno1**, try:
```bash
sudo ip link set down eno1
sleep 5
sudo netplan apply
```

### Install OpenVPN
Just:
```bash
sudo apt-get install openvpn
```
### Configure ROS2 DDS to use the bridge for connections.
Cyclone DDS needs to use **rosbridge** for its connections for discovery to work. To make this happen, you need to set a value for the CYCLONEDDS_URI environment variable to either point to a file, or give the configuration directly. For the direct approach, run this to set it locally and in your default bash startup script:
```bash
export CYCLONEDDS_URI='<General><NetworkInterfaceAddress>rosbridge</></>'
echo "export CYCLONEDDS_URI='$CYCLONEDDS_URI'" >> ~/.bashrc
```
At this point, it might be good to check if ROS2 is still working to a separate machine on the local network. On a separate machine, run:
```bash
ros2 run demo_nodes_cpp listener
```
On the desktop gateway, run:
```bash
ros2 run demo_nodes_cpp talker
```
The listener should here the talker. As another test, if you have Docker installed on the gateway, run:
```bash
docker run -it --rm --net=host --pid=host -e CYCLONEDDS_URI=$CYCLONEDDS_URI osrf/ros:galactic-desktop ros2 run demo_nodes_cpp talker
```
Again the listener on the other machine should hear the talker on the desktop gateway.

### Configure iptables rules for the bridge
When Docker is running, it sets the default acceptance of the FORWARD chain to DENY, and loads the **br_netfilter** kernel module so that layer 2 forwards on the bridge go through the iptables FORWARD chain. The net result is that unless we explictly allow rosbridge forwards in iptables, they will fail when Docker Desktop is started. See [Docker-OpenVPN interactions](/notes/2021/11/docker-openvpn-interactions/) for notes on this issue.

The following script will fix that. (This will also persist the current state of your iptables. If that is not what you want, you probably know what your are doing, and should figure out your preferred way to set and persist this addition to the DOCKER-USERS iptables.)

```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent
sudo iptables -F DOCKER-USER 2> /dev/null || sudo iptables -N DOCKER-USER
sudo iptables -A DOCKER-USER -i rosbridge -o rosbridge -j ACCEPT
sudo iptables -A DOCKER-USER -j RETURN
sudo iptables-save -f /etc/iptables/rules.v4
```

## Cloud Gateway Installation and Configuration

Now we'll switch to the remote Cloud Gateway. Unless otherwise specified, commands in this section are executed on the Cloud machine ssh console. How you open that console depends on how you created the Cloud Gateway. I'm assuming you are starting from a fresh install of Ubuntu 20.04 minimal.

### Software installation (OpenVPN, ROS2, Docker)

Get and run (by sourcing) the install script.
```bash
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain4-install.sh
# Modify any variables in the file if desired before running this.
source ain4-install.sh
```
This script is idempotent, so if something goes wrong you can rerun it multiple times.

## Initialize the EasyRSA public key infrastructure
This script will ask you to set some passwords for the certificate authority, and also ask for a name for that authority. I just use the same password for all requests, and accept the default name for the certificate authority.
```bash
sudo ovpn_initpki
```

### Prepare the client configuration file
Here you create keys for the Desktop Gateway that will connect to this Cloud OpenVPN server, and create the file you will need to login. You'll be asked for the password that you set earlier.
```bash
sudo easyrsa build-client-full ros_local_gateway nopass
sudo ovpn_getclient ros_local_gateway > ros_local_gateway.ovpn
```

### Start and enable OpenVPN using the default openvpn scripts
```bash
sudo systemctl enable openvpn@openvpn
sudo systemctl start openvpn@openvpn
```

You can view the logs for openvpn like this:
```bash
sudo journalctl -f --unit openvpn@openvpn
```
A successful startup looks like this:
```bash
ubuntu@ip-172-31-24-171:~$ sudo journalctl -f --unit openvpn@openvpn
-- Logs begin at Thu 2021-11-04 22:52:33 UTC. --
Nov 04 23:06:30 rosovpn ovpn-openvpn[7774]: /bin/bash -c ip link set master rosbridge dev tap0 && ip link set up tap0 tap0 1500 1654 192.168.255.1 255.255.255.0 init
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: Could not determine IPv4/IPv6 protocol. Using AF_INET
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: Socket Buffers: R=[212992->212992] S=[212992->212992]
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: UDPv4 link local (bound): [AF_INET][undef]:1194
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: UDPv4 link remote: [AF_UNSPEC]
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: GID set to nogroup
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: UID set to nobody
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: MULTI: multi_init called, r=256 v=256
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: IFCONFIG POOL: base=192.168.255.2 size=253, ipv6=0
Nov 04 23:06:31 rosovpn ovpn-openvpn[7774]: Initialization Sequence Completed
```

The IP space should look like this. **rosbridge** and **tap0** should be created, and **tap0** should have "master rosbridge" in its definitions. Also note that rosbridge does not yet have an IP address, since it needs DHCP from the Desktop Gateway network.
```bash
ubuntu@ip-172-31-24-171:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:66:94:fc:80:4d brd ff:ff:ff:ff:ff:ff
    inet 172.31.24.171/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 3167sec preferred_lft 3167sec
    inet6 fe80::66:94ff:fefc:804d/64 scope link 
       valid_lft forever preferred_lft forever
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 22:7c:e3:ac:d3:c7 brd ff:ff:ff:ff:ff:ff
    inet6 fe80::28be:26ff:fea2:e488/64 scope link 
       valid_lft forever preferred_lft forever
4: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:d7:d9:b8:c3 brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether 22:7c:e3:ac:d3:c7 brd ff:ff:ff:ff:ff:ff
    inet6 fe80::207c:e3ff:feac:d3c7/64 scope link 
       valid_lft forever preferred_lft forever
```
Also routes do not yet include rosbridge:
```bash
ubuntu@ip-172-31-24-171:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.16.1     0.0.0.0         UG    100    0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.31.16.0     0.0.0.0         255.255.240.0   U     0      0        0 eth0
172.31.16.1     0.0.0.0         255.255.255.255 UH    100    0        0 eth0
```

## Desktop Gateway Connection to OpenVPN

We now switch back to the Desktop Gateway computer, download the file that contains our OpenVPN configuration, and connect to OpenVPN.

### Download connection file
You need to download the desktop OpenVPN configuration file **ros_local_gateway.ovpn** that we created in previous steps using scp. How you access scp depends on your setup.

As for me, I have setup an SSH configuration called "ros-openvpn" that connects to the Cloud Gateway computer. So I copy the file to a folder on my desktop, from a terminal session on my desktop, using:
```bash
scp ros-openvpn:~/ros_local_gateway.ovpn ./
```
Copy your setup file to a directory on your Desktop Gateway, adapting that **scp** command to your setup.

Now connect to OpenVPN, on a separate terminal, using the file that you just downloaded:
```
sudo openvpn ros_local_gateway.ovpn
```
A successful connection looks like:
```bash
kent@ubutower:~/openvpn$ sudo openvpn ros_local_gateway.ovpn
Thu Nov  4 16:22:52 2021 OpenVPN 2.4.7 x86_64-pc-linux-gnu [SSL (OpenSSL)] [LZO] [LZ4] [EPOLL] [PKCS11] [MH/PKTINFO] [AEAD] built on Jul 19 2021
Thu Nov  4 16:22:52 2021 library versions: OpenSSL 1.1.1f  31 Mar 2020, LZO 2.10
Thu Nov  4 16:22:52 2021 NOTE: the current --script-security setting may allow this configuration to call user-defined scripts
Thu Nov  4 16:22:52 2021 TCP/UDP: Preserving recently used remote address: [AF_INET]44.232.53.80:1194
Thu Nov  4 16:22:52 2021 UDP link local: (not bound)
Thu Nov  4 16:22:52 2021 UDP link remote: [AF_INET]44.232.53.80:1194
Thu Nov  4 16:22:52 2021 WARNING: 'link-mtu' is used inconsistently, local='link-mtu 1573', remote='link-mtu 1574'
Thu Nov  4 16:22:52 2021 WARNING: 'comp-lzo' is present in remote config but missing in local config, remote='comp-lzo'
Thu Nov  4 16:22:52 2021 [44.232.53.80] Peer Connection Initiated with [AF_INET]44.232.53.80:1194
Thu Nov  4 16:22:53 2021 TUN/TAP device tap0 opened
Thu Nov  4 16:22:53 2021 /bin/bash -c ip link set master rosbridge dev tap0 && ip link set up tap0 tap0 1500 1584 192.168.255.2 255.255.255.0 init
Thu Nov  4 16:22:53 2021 WARNING: this configuration may cache passwords in memory -- use the auth-nocache option to prevent this
Thu Nov  4 16:22:53 2021 Initialization Sequence Completed
```
If everything went OK, then the rosbridge on the Cloud Gateway should get an IP address over DHCP. It seems to take about 30 seconds on my system for some reason. But eventually I see, from the Cloud Gateway:
```bash
ubuntu@ip-172-31-20-12:~$ ip a show dev rosbridge
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether c2:9a:4b:fc:93:0f brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.52/24 brd 192.168.0.255 scope global dynamic rosbridge
       valid_lft 7129sec preferred_lft 7129sec
    inet6 fe80::8c62:5bff:fef3:ebd7/64 scope link 
       valid_lft forever preferred_lft forever
```
The link **rosbridge** now should have an ip address from the local network.

From the Desktop Gateway I can ping that address:
```bash
kent@ubutower:~$ ping 192.168.0.52
PING 192.168.0.52 (192.168.0.52) 56(84) bytes of data.
64 bytes from 192.168.0.52: icmp_seq=1 ttl=64 time=23.8 ms
64 bytes from 192.168.0.52: icmp_seq=2 ttl=64 time=12.0 ms
```
As a more severe test, you should also be able to ping from a separate computer located on the same local network:
```bash
kent@uburog:~$ ping 192.168.0.52
PING 192.168.0.52 (192.168.0.52) 56(84) bytes of data.
64 bytes from 192.168.0.52: icmp_seq=1 ttl=64 time=20.9 ms
64 bytes from 192.168.0.52: icmp_seq=2 ttl=64 time=10.3 ms
```
You can even ping the Cloud Gateway by name:
```bash
kent@uburog:~$ ping rosovpn.local
PING rosovpn.local (192.168.0.52) 56(84) bytes of data.
64 bytes from 192.168.0.52 (192.168.0.52): icmp_seq=1 ttl=64 time=10.9 ms
64 bytes from 192.168.0.52 (192.168.0.52): icmp_seq=2 ttl=64 time=10.9 ms
```

### Systemd management of connection

You'll need to have some way to automatically start the connection to the Cloud Gateway. OpenVPN has some standard systemd scripts that allow this.

If you are still running the terminal connection to OpenVPN from your desktop, stop it. Then at the Desktop Gateway:
```bash
sudo cp ros_local_gateway.ovpn /etc/openvpn/client/rosovpn.conf
```
You can now control the connection using **systemctl** unit **openvpn-client@rosovpn** Example:
```bash
kent@ubutower:~/openvpn$ sudo systemctl start openvpn-client@rosovpn
kent@ubutower:~/openvpn$ sudo journalctl --unit openvpn-client@rosovpn
Journal file /var/log/journal/6d23cf51dee346fa9ec4f0a934322fbb/user-1000@0005cf1f8f58b5a1-0bfe030dfc1d00ea.journal~ is truncated, ignoring file.
-- Logs begin at Tue 2021-08-24 13:17:02 PDT, end at Fri 2021-11-05 15:56:54 PDT. --
Nov 05 15:50:15 ubutower.dryrain.org systemd[1]: Starting OpenVPN tunnel for ros...
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: OpenVPN 2.4.7 x86_64-pc-linux-gnu [SSL (OpenSSL)] [LZO] [LZ4] [EPOLL] [PKCS11] [MH/PKTINFO] [AEAD] built on Jul 19 2021
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: library versions: OpenSSL 1.1.1f  31 Mar 2020, LZO 2.10
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: NOTE: the current --script-security setting may allow this configuration to call user-defined scripts
Nov 05 15:50:15 ubutower.dryrain.org systemd[1]: Started OpenVPN tunnel for ros.
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: TCP/UDP: Preserving recently used remote address: [AF_INET]44.232.53.80:1194
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: UDP link local: (not bound)
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: UDP link remote: [AF_INET]44.232.53.80:1194
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: WARNING: 'link-mtu' is used inconsistently, local='link-mtu 1573', remote='link-mtu 1574'
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: WARNING: 'comp-lzo' is present in remote config but missing in local config, remote='comp-lzo'
Nov 05 15:50:15 ubutower.dryrain.org openvpn[6890]: [44.232.53.80] Peer Connection Initiated with [AF_INET]44.232.53.80:1194
Nov 05 15:50:17 ubutower.dryrain.org openvpn[6890]: TUN/TAP device tap0 opened
Nov 05 15:50:17 ubutower.dryrain.org openvpn[6890]: /bin/bash -c ip link set master rosbridge dev tap0 && ip link set up tap0 tap0 1500 1584 192.168.255.2 255.255.255.0 init
Nov 05 15:50:17 ubutower.dryrain.org openvpn[6890]: WARNING: this configuration may cache passwords in memory -- use the auth-nocache option to prevent this
Nov 05 15:50:17 ubutower.dryrain.org openvpn[6890]: Initialization Sequence Completed
kent@ubutower:~$ ping 192.168.0.52
PING 192.168.0.52 (192.168.0.52) 56(84) bytes of data.
64 bytes from 192.168.0.52: icmp_seq=1 ttl=64 time=10.1 ms
64 bytes from 192.168.0.52: icmp_seq=2 ttl=64 time=10.0 ms
```
Of course you can also enable this to happen automatically on reboot:
```bash
sudo systemctl enable openvpn-client@rosovpn
```
And now the ultimate end-to-end test: ROS2 in Docker on a second computer (that is, the real or simulated robot), connecting to ROS2 on Docker in the Cloud Gateway. Here's the result:

On the second computer:
```bash
docker run -it --rm --net=host --pid=host \
  osrf/ros:galactic-desktop ros2 \
  run demo_nodes_cpp listener
```
On the Cloud Gateway, the AWS test machines I use don't have much space, so I have to make a smaller Docker image. Run this script to make a demo image with just ROS2 and the demo package:
```bash
cat <<EOF > Dockerfile
FROM ros:galactic-ros-base-focal
RUN apt update
RUN apt install -y ros-galactic-demo-nodes-cpp
EOF

docker build -t rosdemo ./
```
Now run the talker:
```bash
docker run -it --rm --net=host --pid=host \
  -e CYCLONEDDS_URI=$CYCLONEDDS_URI \
  rosdemo \
  ros2 run demo_nodes_cpp talker
```
The listener on the second system should hear the ROS2 node running on Docker on the Cloud Gateway.

## Reference Notes

0. iptables interactions: [Docker-OpenVPN interactions](/notes/2021/11/docker-openvpn-interactions/)
0. Netplan to configure bridge: [Persisting a Docker bridge](/notes/2021/10/persisting-bridge/) and [Netplan Bridge Configuration](/notes/2021/10/netplan-bridge-config/)
0. Need to down ethernet prior to **netplan apply**: [Netplan Apply Failure with tap0 and eno1](/notes/2021/11/netplan-bridge-add-fails-eno1/)
