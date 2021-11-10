---
layout: single
title:  "Adventures in ROS2 Networking 3 - ROS2 over OpenVPN Gateway"
tags: AWS networking OpenVPN
excerpt: "Tests of ROS2 on an OpenVPN gateway with fixes"
permalink: adventures-in-ros2-networking-3
---
## Introduction

In our [previous post](/adventures-in-ros2-networking-2) we installed and configured an OpenVPN gateway with ROS2 installed, but we did not test ROS2. Here we will test communication over the gateway with ROS2 and suggest configuration changes when we have problems.

Here's our goal for this post. We have added to the configuration of the previous post ROS2 running on the Desktop Gateway (including possibly over Docker), and on the Cloud Gateway:

![ROS2 Over Desktop to Cloud OpenVPN Gateway](/assets/images/OpenVpnGateways3.png "ROS2 Over Desktop to Cloud OpenVPN Gateway")

To run this demo, you'll need to start with the machine as configured from the end of that post. We'll use the term "desktop" to refer to commands run from your local machine (the "Desktop Gateway"), and "remote" for the Cloud Gateway with ROS2 and OpenVPN installations.

## Start and test OpenVPN

On the remote, make sure that openvpn is not being run by systemd, then run in a separate terminal:
```bash
sudo systemctl stop openvpn@openvpn
sleep 5
sudo ovpn_run
```
On the desktop in a separate terminal, from the folder where the configuration file ros_desktop.ovpn is, run openvpn:
```bash
sudo openvpn ros_desktop.ovpn
```

Confirm operation of openvpn by pinging. In the examples that follow, replace the ip address with the actual private ip addresses for the tap0 interface as discovered by `ip a`. By default, the Desktop Gateway address should be **192.168.255.2** and the Cloud Gateway **192.168.255.1**.

On the desktop:
```bash
kent@ubutower:~$ ping 192.168.255.1
PING 192.168.255.1 (192.168.255.1) 56(84) bytes of data.
64 bytes from 192.168.255.1: icmp_seq=1 ttl=64 time=24.3 ms
64 bytes from 192.168.255.1: icmp_seq=2 ttl=64 time=12.2 ms
```

On the remote:
```bash
ubuntu@ip-172-31-11-150:~$ ping 192.168.255.2
PING 192.168.255.2 (192.168.255.2) 56(84) bytes of data.
64 bytes from 192.168.255.2: icmp_seq=1 ttl=64 time=11.8 ms
64 bytes from 192.168.255.2: icmp_seq=2 ttl=64 time=12.2 ms
```

## Naive ROS2 test
Make sure that ROS2 is runnable from both the desktop and remote. The install of the remote from the previous post should have added entries to .bashrc to source ROS2. Do whatever you need to do on the desktop to also run ROS2 there. You'll also need the ros2 package **demo_nodes_cpp** which we also used in [part 1](/adventures-in-ros2-networking-1) of this series.

Start a listener on the desktop:
```bash
kent@ubutower:~/openvpn$ ros2 run demo_nodes_cpp listener
1635184098.531129 [0]   listener: using network interface eno1 (udp/192.168.0.59) selected arbitrarily from: eno1, tap0
```

Start a talker on the remote:
```bash
ubuntu@openvpn:~$ ros2 run demo_nodes_cpp talker
1635184161.721264 [0]     talker: using network interface eth0 (udp/172.31.17.172) selected arbitrarily from: eth0, tap0
[INFO] [1635184162.742017201] [talker]: Publishing: 'Hello World: 1'
[INFO] [1635184163.741990662] [talker]: Publishing: 'Hello World: 2'
[INFO] [1635184164.741980461] [talker]: Publishing: 'Hello World: 3'
```

The talker talks, but the listener hears nothing. So far, so bad.

## Configuring CycloneDDS

The startup comments give us a hint of the issue. Both ROS2 runs are using interfaces with local private IP addresses that are not visible to the other system. Fortunately, we can [fix this](https://dds-demonstrators.readthedocs.io/en/latest/Teams/1.Hurricane/setupCycloneDDS.html "Setup Guide for CycloneDDS"). We followed the same appoach in [part 1 of this series](/adventures-in-ros2-networking-1 "Adventures in ROS2 Networking 1 - Local")

### File Option
Here is a basic CycloneDDS configuration file that forces it to use the tap0 interface:
```
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>tap0</NetworkInterfaceAddress>
        </General>
    </Domain>
  </CycloneDDS>
```
You can also download this [here](https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/cyclonedds-tap0.xml)

Add that file to both your remote and desktop by running on each:
```bash
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/cyclonedds-tap0.xml
```

Now restart the talker and listener using that file. On the remote:
```bash
ubuntu@openvpn:~$ CYCLONEDDS_URI=file://$PWD/cyclonedds-tap0.xml ros2 run demo_nodes_cpp talker
[INFO] [1635185693.807050508] [talker]: Publishing: 'Hello World: 1'
[INFO] [1635185694.807034884] [talker]: Publishing: 'Hello World: 2'
[INFO] [1635185695.807017463] [talker]: Publishing: 'Hello World: 3'
[INFO] [1635185696.806971836] [talker]: Publishing: 'Hello World: 4'
[INFO] [1635185697.807008435] [talker]: Publishing: 'Hello World: 5'
[INFO] [1635185698.806973966] [talker]: Publishing: 'Hello World: 6'
```
On the desktop:
```bash
kent@ubutower:~/openvpn$ CYCLONEDDS_URI=file://$PWD/cyclonedds-tap0.xml ros2 run demo_nodes_cpp listener
[INFO] [1635185696.806410910] [listener]: I heard: [Hello World: 4]
[INFO] [1635185697.806471912] [listener]: I heard: [Hello World: 5]
[INFO] [1635185698.806514409] [listener]: I heard: [Hello World: 6]
```

Success!

### Literal Option

This variable **CYCLONEDDS_URI** can either contain a URI reference to a configuration file for Cyclone DDS, or it can contain the text of that configuration directly. When the configuration changes are simple, as they are in these demos, the text form is a little easier. The simple text version to use the **tap0** interface is:
```bash
CYCLONEDDS_URI="<General><NetworkInterfaceAddress>tap0</></>"
```

Try that alternate form. On the Desktop,
```bash
CYCLONEDDS_URI="<General><NetworkInterfaceAddress>tap0</></>" ros2 run demo_nodes_cpp talker
```

and on the remote:
```bash
CYCLONEDDS_URI="<General><NetworkInterfaceAddress>tap0</></>" ros2 run demo_nodes_cpp listener
```

This should also work.

## Using the ROS2 Multicast Test (or Not!)

ROS2 has a multicast command that is supposed to test connectivity. Let's try that command with our fix:

On the desktop:
```bash
kent@ubutower:~/openvpn$ CYCLONEDDS_URI=file://$PWD/cyclonedds-tap0.xml ros2 multicast receive
Waiting for UDP multicast datagram...
```

On the remote:
```bash
ubuntu@openvpn:~$ CYCLONEDDS_URI=file://$PWD/cyclonedds-tap0.xml ros2 multicast send
Sending one UDP multicast datagram...
```

The multicast message is not heard on the desktop. I'm not sure what is going on here, but I suspect that this multicast command is choosing the wrong interface. But the takehome message is that the ros2 multicast command is not a definitive way to confirm or deny connectivity between systems.

## ROS2 on Docker Container on Desktop

Previously in this post, I did not have Docker Desktop running, which was by design since it can cause [extra issues](/notes/2021/11/docker-openvpn-interactions/ "Docker-OpenVPN interactions") that we will deal with in the next post. But we can get a simple configuration running without much additional effort.

We'll run ROS2 on a Docker container on the desktop, but run ROS2 on the host on the remote. Since the naive ROS2 test with hosts failed, a naive Docker run will also fail, even with `--net=host --pid=host`. BUt let's see what it does. Go ahead and start a talker on the remote using the same as **CYCLONEDDS_URI** configuration as before:

```bash
CYCLONEDDS_URI="<General><NetworkInterfaceAddress>tap0</></>" ros2 run demo_nodes_cpp talker
```

Make sure that Docker Desktop is running on the host (`sudo systemctl start docker` if needed) and run a listener, with openVPN connected:
```bash
kent@ubutower:~$ docker run -it --rm --net=host --pid=host osrf/ros:galactic-desktop ros2 run demo_nodes_cpp listener
1636494806.869057 [0]   listener: using network interface eno1 (udp/192.168.0.55) selected arbitrarily from: eno1, tap0, docker0
```
The interface being used does not connect to the OpenVPN gateway, as we saw earlier. But we can fix this using the literal version of the **CYCLONEDDS_URI** environment variable. On the desktop:
```bash
export CYCLONEDDS_URI="<General><NetworkInterfaceAddress>tap0</></>"
docker run -it --rm --net=host --pid=host -e CYCLONEDDS_URI=$CYCLONEDDS_URI osrf/ros:galactic-desktop ros2 run demo_nodes_cpp listener
```

Now the listener should hear the talker running on the remote. We did not install Docker on the remote yet, but if we did it would also work using the same configuration of **CYCLONEDDS_URI**.

What does not work yet is connecting a ROS2 node running on the remote, with a ROS2 node running on a second local computer (such as the actual robot). There's a few more issues still to deal with, which we will accomplish in the next post.
