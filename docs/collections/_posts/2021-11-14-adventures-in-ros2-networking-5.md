---
layout: single
title:  "Adventures in ROS2 Networking 5 - Second Instance in Cloud"
tags: AWS networking OpenVPN
excerpt: "We add a tunnel so that ROS2 nodes ona second cloud instance work with the cloud gateway"
permalink: adventures-in-ros2-networking-5
---
## Introduction

In our [previous post](/adventures-in-ros2-networking-4 "Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway") we connected an OpenVPN cloud gateway to a local network. Here we are going to add a second cloud processor that also uses the gateway:

![Robot and OpenVPN Gateways](/assets/images/OpenVpnGateways5.png)

There are no changes to the setup of the **Desktop Gateway** or **Robot** in this post compared to the previous, so use that post to see how to configure the **Desktop Gateway**. (The robot needs no configuration changes to work with the **Desktop Gateway** as designed.)

## Overall Plan

Previously we used DHCP to assign an IP address to the **Cloud Gateway** bridge that matches the robot network. Here instead we will use a static IP address. This makes setup a little easier and more reliable, and id typically how you would setup critical instances in a small network anyway.

The main issue here is how to get the ROS2 on the **Second Instance** to communicate with the **Cloud Gateway**, and thereby also with the **Robot**. In the local network, this just worked. Unfortunately typical cloud virtual networks (and specifically Amazon Web Services where I did my tests) do not work like normal local networks at layer 2 where the DDS multicast node discovery is done. When [I tried](/notes/2021/11/ROS2-on-AWS/ "ROS2 nodes on multiple AWS machines") adding a second node, it did not work. AWS claims [it can use multicast](https://docs.aws.amazon.com/vpc/latest/tgw/how-multicast-works.html "Multicast routing") if you use a "transit gateway" but that adds additional cost and complexity that I did not attempt.

Ultimately my [various experiments](/notes/2021/11/ain5-scripts/ "AIN5 - Trying to get the scripts working") succeeded in getting a second AWS instance to work by setting up a layer 2 point-to-point **gretap** tunnel between the **Second Instance** and the **Cloud Gateway**. This works OK, but you need to setup a separate **gretap** tunnel between each additional AWS instance and the gateway.

Type **vxlan** tunnels are supposed to allow point-to-multipoint tunnels which could scale better, but unfortunately they rely on multicast for discovery of additional instances. And that is our main issue - multicast does not work! So point-to-point **gretap** tunnels it is.

A few weeks ago I didn't know what a **gretap** tunnel was. In case it is new to you, here is a brief explanation. A **gretap** tunnel is a Linux networking technology that lets you add a link in your Linux system's network that acts just like an ethernet port, but instead of being a physical interface, it is connected using a layer 3 protocol (called GRE) to another **gretap** device on another Linux system. It's like you have second ethernet cards in each system that are connected to each other, but instead of being connected with a wire directly, they are connected via IP using a different network connection. 

As we do this setup, we are going to specify various IP addresses. These exist in a variety of networks. Let me clear what those are:

- the **Cloud** addresses are private IPs that your various cloud instances use to communicate with each other. I typically assign those manually, but that is not really necessary, you can use the auto assigned addresses if you want. But you need to know what they are to setup the tunnel.
- the **VPN** addresses are private IPs that are used by the **Robot** and are extended to the cloud with the OpenVPN gateways. Typically these are the addresses on you local network.
- the **Gateway Public** address is a public IP address that is assigned to the **Cloud Gateway** so you can connect to it over the internet. There will typically also be public ip addresses for the **Second Instance**, but it is only needed for configuration. (You could use the **Cloud Gateway** as a bastion host to connect to the **Second Instance** if you want).
- **OpenVPN** assigns private IP addresses in 192.168.255.0/24 by default to the two endpoints of its tunnel, but we do not use those addresses in this configuration.

To use the configurations in this post, you will need two cloud instances in the same private subnet. In the examples, the private network is **172.31.1.0/24**. Cloud security must be setup to allow, in addition to the open-to-the-world OpenVPN, ping, and ssh used in the [previous post](/adventures-in-ros2-networking-4 "Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway"), **GRE** between the cloud instances. **GRE** is an IP protocol (that is, at the same level as **TCP**, **UDP**, and **ICMP**) with protocol number 47. Here is a set of inbound rules used in the examples from AWS:

![Inbound rules including GRE](/assets/images/gre-security-group.png)

Otherwise, the requirements are the same as in the previous post.

## Desktop Gateway Setup

For the setup of the **Desktop Gateway** there wlll be no changes in the configuration from the previous post. So you need to complete the [**Desktop Gateway** setup section](/adventures-in-ros2-networking-4#desktop-gateway-setup "Desktop Gateway Setup") from that post, but if you have done that you do not need to repeat it.

As part of the installation of the **Cloud Gateway** in this example, we will re-create the security keys. That means that the file **ros_local_gateway.ovpn** that was created in the previous post will not be valid for this example. We will re-create it and download a new version. 

## Cloud Gateway Installation and Configuration

### Required IP Address Definitions

When we do the installations for the two cloud instances, you will need to define 4 IP addresses for use by the installation script. You can define them using bash variables prior to starting the installation, for example with input like this to bash:
```bash
ROSVPN_GATEWAY_CLOUD_IP=172.31.1.11
ROSVPN_SECOND_CLOUD_IP=172.31.1.12
ROSVPN_GATEWAY_VPN_IP=192.168.0.130/24
ROSVPN_SECOND_VPN_IP=192.168.0.131/24
```
or you can start the install scripts, and you will be prompted to input the needed variables. Here are their meanings:
- ROSVPN_GATEWAY_CLOUD_IP: The IP address of the **Cloud Gateway** in the **Cloud** private subnet
- ROSVPN_SECOND_CLOUD_IP: The IP address of the **Second** instance in the **Cloud** private subnet

The next two addresses are in the subnet of your local (robot) network. Generally you choose them yourself, finding addresses that do not conflict with any in the local network.
- ROSVPN_GATEWAY_VPN_IP: The IP address (in [CIDR notation](https://en.wikipedia.org/wiki/Classless_Inter-Domain_Routing#CIDR_notation "Wikipedia CIDR notation description")) of the **CLOUD GATEWAY** in the **VPN** subnet.
- ROSVPN_SECOND_VPN_IP: The IP address (in CIDR notation) of the **SECOND** instance in the **VPN** private subnet

(Other than the above IP address definition, the instructions in this section are identical to those from the [previous post](/adventures-in-ros2-networking-4#cloud-gateway-installation-and-configuration "Cloud Gateway Installation and Configuration"). The difference from that installation is just a slightly different install script.)

Unless otherwise specified, commands in this section are executed on the **Cloud Gateway** machine ssh console. How you open that console depends on how you created the Cloud Gateway. I'm assuming you are starting from a fresh install of Ubuntu 20.04 minimal.

### Software installation (OpenVPN, ROS2, Docker)

Get and run (by sourcing) the install script.
```bash
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain5-gw-install.sh
source ain5-gw-install.sh
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
```
then
```bash
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
ubuntu@ip-172-31-1-11:~$ sudo journalctl -f --unit openvpn@openvpn
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

The IP space should look like this. **rosbridge**, **gre1@NONE**, and **tap0** should be created.
```bash
ubuntu@ip-172-31-1-11:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:3f:8b:a5:02:51 brd ff:ff:ff:ff:ff:ff
    inet 172.31.1.11/24 brd 172.31.1.255 scope global dynamic eth0
       valid_lft 2243sec preferred_lft 2243sec
    inet6 fe80::3f:8bff:fea5:251/64 scope link 
       valid_lft forever preferred_lft forever
3: gre0@NONE: <NOARP> mtu 1476 qdisc noop state DOWN group default qlen 1000
    link/gre 0.0.0.0 brd 0.0.0.0
4: gretap0@NONE: <BROADCAST,MULTICAST> mtu 1462 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
5: erspan0@NONE: <BROADCAST,MULTICAST> mtu 1450 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
6: gre1@NONE: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1462 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 1000
    link/ether ea:0d:2c:4e:a7:4d brd ff:ff:ff:ff:ff:ff
7: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1462 qdisc noqueue state UP group default qlen 1000
    link/ether be:24:32:cc:bd:bc brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::40aa:37ff:fe9f:1a55/64 scope link 
       valid_lft forever preferred_lft forever
8: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:0e:ec:ef:40 brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
9: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether be:24:32:cc:bd:bc brd ff:ff:ff:ff:ff:ff
    inet6 fe80::bc24:32ff:fecc:bdbc/64 scope link 
       valid_lft forever preferred_lft forever
```
Note that both the **tap0** tunnel to the **Desktop Gateway**, as well as the **gre1@NONE** tunnel to the **Second** cloud instance have "master rosbridge" in their details, so **rosbridge** is serving as a layer 2 router to send ethernet packets to the correct device.

Routes include **rosbridge** since we gave it an IP address:
```bash
ubuntu@ip-172-31-1-11:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.1.1      0.0.0.0         UG    100    0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.31.1.0      0.0.0.0         255.255.255.0   U     0      0        0 eth0
172.31.1.1      0.0.0.0         255.255.255.255 UH    100    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     0      0        0 rosbridge
```

## Desktop Gateway Connection to OpenVPN

We now switch back to the **Desktop Gateway** computer, download the file that contains our OpenVPN configuration, and connect to OpenVPN.

### Download connection file
You need to download the desktop OpenVPN configuration file **ros_local_gateway.ovpn** that we created in previous steps using scp. How you access scp depends on your setup.

As for me, I have setup an SSH configuration called "ros-openvpn" that connects to the Cloud Gateway computer. So I copy the file to a folder on my desktop, from a terminal session on my desktop, using:
```bash
scp ros-openvpn:~/ros_local_gateway.ovpn ./
```
Copy your setup file to a directory on your Desktop Gateway, adapting that **scp** command to your setup. Now connect to OpenVPN, on a separate terminal, using the file that you just downloaded:
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

From the Desktop Gateway I can ping the address ROSVPN_GATEWAY_VPN_IP of the **Cloud Gateway**:
```bash
kent@ubutower:~$ ping 192.168.0.130
PING 192.168.0.130 (192.168.0.130) 56(84) bytes of data.
64 bytes from 192.168.0.130: icmp_seq=1 ttl=64 time=23.8 ms
64 bytes from 192.168.0.130: icmp_seq=2 ttl=64 time=12.0 ms
```

## Second Processor Installation and Configuration

Now we'll install the **Second** cloud instance that we will use as a second place we can run ROS2 nodes. The requirements of this are the generally the same as for the **Cloud Gateway**. You don't really need the OpenVPN port available to it, and you don't really need a permanent IP address, just a temporary for initial configuration. This installation will use the same IP address definitions discussed [above](#required-ip-address-definitions "Required IP Address Definitions"). Either enter them before installation, or you will be asked to define them in the installation script.

### Installation
Starting from a minimal Ubuntu installation, download and source the installation script:
```
wget https://github.com/rosdabbler/rosdabbler.github.io/raw/main/scripts/ain5-vx-install.sh
source ain5-vx-install.sh
```
That's all it takes. 

### Second Testing
With the OpenVPN gateway connected, and a second system called **uburog** on my local network that simulates a robot, I can ping it by name from the cloud **Second**:
```bash
ubuntu@rosvpn-second:~$ ping -n uburog.local
PING uburog.local (192.168.0.83) 56(84) bytes of data.
64 bytes from 192.168.0.83: icmp_seq=1 ttl=64 time=11.0 ms
64 bytes from 192.168.0.83: icmp_seq=2 ttl=64 time=11.7 ms
```
Now let me do a full end-to-end Docker test. On the **Second** build a small image that can run the demo:
```bash
cat <<EOF > Dockerfile
FROM ros:galactic-ros-base-focal
RUN apt update
RUN apt install -y ros-galactic-demo-nodes-cpp
EOF

docker build -t rosdemo ./
```
Start a talker there:
```bash
docker run -it --rm --net=host --pid=host \
  -e CYCLONEDDS_URI=$CYCLONEDDS_URI \
  rosdemo \
  ros2 run demo_nodes_cpp talker
```

On the local robot or an additional system simulating that robot, run a Docker listener:
```bash
docker run -it --rm --net=host --pid=host \
  osrf/ros:galactic-desktop ros2 \
  run demo_nodes_cpp listener
```
The docker listener on the simulated robot should hear the talker from the cloud **Second**. That was our ultimate goal.

## Parting Comments
And so I bring this series to an end, at least for awhile. Ultimately, I will probably run my robot on a separate network from the local computer, so that I can define IP addresses for the cloud computers that are not dependent on a particular local network. I did [some experiments](/notes/2021/10/docker-bridge-reuse/ "Experiments reusing the docker bridge with openvpn") that show me this is feasible.
