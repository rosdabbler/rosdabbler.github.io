---
title: "Experiments reusing the docker bridge with openvpn"
layout: archive
author_profile: true
tags: [docker, openvpn]
date: 2021-10-22
---

I believe the fully using Docker and OpenVPN together will require that I have a single bridge that is
used for both OpenVPN as well as docker. Here I'll document my experiments.

## Create a docker bridge with a name

Since I will be needing to manipulate the docker bridge, it should be created with a name. Fortunately
Docker supports this. Create a named docker bridge:

```bash
docker network create \
  --subnet 10.231.101.0/24 \
  --gateway 10.231.101.10 \
  --ip-range 10.231.101.128/26 \
  --opt com.docker.network.bridge.name=docker_openvpn \
ovpn
```

The bridge starts down as ```ip a``` shows. If I start a simple container though it goes up.
```bash
docker --t --rm --net ovpn busybox
```
It goes down when the container is stopped.

I want to add my ethernet connection eno1 to this bridge. I'll need to use the brctl command, which comes from bridge-utils
```bash
sudo apt install bridge-utils
```

I know from earlier experiments that routing is going to be an issue. Here's how routing starts:
```bash
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    100    0        0 eno1
10.231.101.0    0.0.0.0         255.255.255.0   U     0      0        0 docker_openvpn
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 docker0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```

I want my ethernet interface eno1 to be in the docker_openvpn bridge.
```bash
sudo brctl addif docker_openvpn eno1
```

Start openvpn on the server, and login locally. I'm using the script from [adventures-in-ros2-networking-2](/adventures-in-ros2-networking-2)

Login to the remote server with ```ssh ros-openvpn``` and start openvpn with ```sudo ovpn_run```.

On the desktop, connect to openvpn with ```sudo openvpn ros_desktop.ovpn``` That creates a **tap0** interface on the desktop. I want to add this to the docker bridge:
```bash
sudo brctl addif docker_openvpn tap0
```

That left issues in my routing table with duplicate routes:
```bash
kent@ubutower:~/openvpn$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    20100  0        0 eno1
10.231.101.0    0.0.0.0         255.255.255.0   U     0      0        0 docker_openvpn
10.231.101.0    0.0.0.0         255.255.255.0   U     0      0        0 tap0
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 docker0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```
This does not cause any issues yet, but it will. I am going to want to add a route to a robot at some point, and I do not want that to go through tap0 to the cloud, but let the bridge manage routing. So I need to remove the 10.231.101.0 route to tap0.
```bash
kent@ubutower:~$ sudo ip route del 10.231.101.0/24 dev tap0
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    20100  0        0 eno1
10.231.101.0    0.0.0.0         255.255.255.0   U     0      0        0 docker_openvpn
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 docker0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```
OK that did it, I can successfully ```ping 10.231.101.1``` from the desktop. But I need to figure out later if there is an openvpn or linux option to prevent that route from being automatically added.

How about ROS2? On the desktop, I created a dds config file **bridge-dds** to use the bridge:

```
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:s>
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>docker_openvpn</NetworkInterfaceAddress>
        </General>
    </Domain>
  </CycloneDDS>
```

I started a talker on the remote:
```bash
$ CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run demo_nodes_cpp talker
```
I started a listener on the desktop:
```bash
$ CYCLONEDDS_URI=file://$PWD/bridge-dds.xml ros2 run demo_nodes_cpp listener
```

That worked:
```bash
kent@ubutower:~/openvpn$ CYCLONEDDS_URI=file://$PWD/bridge-dds.xml ros2 run demo_nodes_cpp listener
[INFO] [1635277839.214475042] [listener]: I heard: [Hello World: 1]
[INFO] [1635277840.213716849] [listener]: I heard: [Hello World: 2]
[INFO] [1635277841.213704470] [listener]: I heard: [Hello World: 3]
```
but not the first time that I tried it. When it failed, ping was also failing, so I
must have made some error. Need to get this all automated.

## ROS2 tests under Docker
The standard ROS2 desktop image osrf/ros:galactic-desktop does not have all of the networking utilities installed, so let me create a basic Dockerfile with this.

The Dockerfile:
```dockerfile
FROM osrf/ros:galactic-desktop

# install ros packages
RUN apt-get update

# extra stuff
RUN apt-get install -y --no-install-recommends \
  iproute2 \
  bridge-utils \
  iputils-ping \
  dnsutils \
  nano \
  less
```

On the desktop, build it, and start it:
```bash
docker build -t 'rosnet' .
docker run -it --rm --net ovpn rosnet bash
```

Now I am going to start two listeners. One is run on the desktop directly:
```bash
kent@ubutower:~/openvpn$ CYCLONEDDS_URI=file://$PWD/bridge-dds.xml ros2 run demo_nodes_cpp listener
```

A second is run on the openvpn gateway:
```bash
ubuntu@openvpn:~$ CYCLONEDDS_URI=file://$PWD/cyclonedds.xml ros2 run demo_nodes_cpp listener
```

Now I'll start a docker container with ROS2 on the desktop
```bash
kent@ubutower:~/openvpn$ dr --net ovpn -h rosnet rosnet bash
```

We'll run a talker here. The docker container only has a single interface, so we don't need to configure DDS.
```bash
root@rosnet:/# ros2 run demo_nodes_cpp talker
[INFO] [1635280872.456529455] [talker]: Publishing: 'Hello World: 1'
[INFO] [1635280873.456487733] [talker]: Publishing: 'Hello World: 2'
[INFO] [1635280874.456404643] [talker]: Publishing: 'Hello World: 3'
```

Hmmm, it worked the first time I tried it, but not this time when I tried to document. I see that the link seems to be restarting due to inactivity. Perhaps that is causing some issues. OK the desktop has lost its tap0 interface, and the terminal where I was running the openvpn server is dead. Let me instead run that as a daemon using systemctl. On the remote:
```bash
sudo systemctl start openvpn@openvpn
sudo journalctl -f --unit openvpn@openvpn
```

Connect to openvpn on the desktop:
```bash
sudo openvpn ros_desktop.ovpn 
```

Now I have to repeat the routing and bridging fixes on the desktop
```bash
sudo brctl addif docker_openvpn tap0
sudo ip route del 10.231.101.0/24 dev tap0
```

Now if I run listeners on the desktop and on the remote, they hear the talker that is being run from the desktop Docker.

I installed openssh-server on the ros Docker. Tried to start with systemctl, it complained. I guess that image does not use it! Swell. Started from init.d, worked. Could connect but not login as root. I probably need to setup another user with keys.

I'm going to redo the Docker file as follows:

```dockerfile
FROM ros:galactic-ros-base-focal

# install ros packages
RUN apt-get update

# ROS for demo
RUN apt-get install -y ros-galactic-demo-nodes-cpp

# extra stuff
RUN apt-get install -y --no-install-recommends \
  iproute2 \
  bridge-utils \
  iputils-ping \
  dnsutils \
  nano \
  less
```

I tried that, it also does not seem to use systemd. Well I see all of the systemd stuff, but no systemctl. I googled a bit, and I guess that is by design.

## Support Mechanisms to do my Network Munging

### Get Network Manager to leave ethernet connection in bridge

I confirmed through logging that a script at kent@ubutower:/etc/NetworkManager/dispatcher.d/pre-up.d runs when the ethernet interface comes up. How about openvpn? ... Yes! 

So what runs when openvpn starts and stops on the client? Checking the [manual](https://manpages.ubuntu.com/manpages/focal/en/man8/openvpn.8.html), here are some interesting options:
- --up cmd (Run command cmd after successful TUN/TAP device open (pre --user UID change).)
- --route-pre-down (Executed right before the routes are removed.)

"--ifconfig-noexec" prevents an IP address for tun0, and stops the route being added, but the route is started down. I will need a script to add it to thebridge, and set ip up. When I do that, ros2 demo nodes communicate. Let me see if I can automate all that through the config.

I modified the .ovpn script for the client, and it does all that is needed (does not add route, adds tun0 to bridge) with this preamble prior to the key:
```
client
nobind
dev tap
remote-cert-tls server
ifconfig-noexec
persist-tun
script-security 2
route-up "/bin/bash -c 'brctl addif docker_openvpn tap0 && ip link set tap0 up'"
remote 44.232.53.80 1194 udp
```

I need to modify the server config instructions to make this work automatic.

---
References:

https://blog.oddbit.com/post/2018-03-12-using-docker-macvlan-networks/

