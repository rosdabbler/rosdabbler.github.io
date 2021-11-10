---
layout: single
title:  "Adventures in ROS2 Networking 1 - Local"
tags: networking Docker
permalink: adventures-in-ros2-networking-1
excerpt: "This is the first post in a series that will explore networking issues with ROS2. This first post shows basic things that Just Work."
---
## Introduction

This series of blog posts will explore ways to configure ROS2 networks in a distributed environment.

I'm getting ready to convert my robot project from ROS1 to ROS2, as well as expand the way  nodes are run to include docker containers and various distributed processors. The possible universe of processors include a local robot, my desktop Ubuntu system, as well as a cloud provider. I may want to run nodes on the host, as well as in Docker containers in each environment. A processor in the cloud will be connected to the desktop and robot by a VPN. Here's where we are headed:

![Robot and OpenVPN Gateways](/assets/images/OpenVpnGateways.png "Robot and OpenVPN Gateways")

ROS2 presents some challenges operating in a distributed manner due to the fact that node discovery requires multicast packets, and node-to-node communication can occur on a wide range of ports. I'll try to present concrete ways to accomplish various networking tasks with different combinations of node execution environments.

This first post deals with basic configurations. You might skip it if you are fluent in ROS2 and Docker.

## Preliminaries

These adventures will require that you have both [ROS2](https://docs.ros.org/en/galactic/Installation.html) and [Docker Desktop](https://docs.docker.com/get-docker/) installed on your main machine. I'll also assume you are using an Ubuntu version that supports ROS2 galactic (or later) unless otherwise specified. I'll assume you know how to source ROS2 in your environment.

Although these instructions should work in a variety of environments, my desktop environment is:
```bash
$ lsb_release -d
Description:	Ubuntu 20.04.3 LTS
$ docker -v
Docker version 20.10.8, build 3967b7d
$ echo $ROS_DISTRO
galactic
```

## Simple Configurations that Just Work
Before we start to deal with complexities, let's try a few things that just work (or work with simple tweaks). For testing purposes, we'll be using the talker and listener demos available in the demo_nodes_cpp package. They are available in most ROS2 variants, but if not can be installed with ```sudo apt install ros-galactic-demo-nodes-cpp```.

### Two nodes on the host
These work just fine when run on a single system. To demo, open up two bash shell sessions. Enter in 1:
```bash
ros2 run demo_nodes_cpp listener
```
and in the other:
```bash
ros2 run demo_nodes_cpp talker
```
The results should be like this:

Talker:
```bash
$ ros2 run demo_nodes_cpp talker
1634274392.947700 [0]     talker: using network interface eno1 (udp/192.168.0.59) selected arbitrarily from: eno1, virbr0, docker0
[INFO] [1634274393.952837498] [talker]: Publishing: 'Hello World: 1'
[INFO] [1634274394.952768359] [talker]: Publishing: 'Hello World: 2'
[INFO] [1634274395.952737612] [talker]: Publishing: 'Hello World: 3'
```
Listener:
```bash
$ ros2 run demo_nodes_cpp listener
1634274374.428509 [0]   listener: using network interface eno1 (udp/192.168.0.59) selected arbitrarily from: eno1, virbr0, docker0
[INFO] [1634274393.953399415] [listener]: I heard: [Hello World: 1]
[INFO] [1634274394.953033668] [listener]: I heard: [Hello World: 2]
[INFO] [1634274395.952884067] [listener]: I heard: [Hello World: 3]
```
Use Ctrl-C to exit from these programs.

This Just Works on at least my system. But the warning message "using network interface ... selected arbitrarily", which appears because this system has Docker Desktop running, hints at problems we'll have to deal with later. 

### Two Nodes in Separate Containers
Another configuration that Just Works is multiple docker containers running on the same system and network. We'll use a released docker image that includes the demo_nodes_cpp package.

Open two terminals. In the first, start the listener:
```bash
docker run -it --rm osrf/ros:galactic-desktop \
  ros2 run demo_nodes_cpp listener
```

In the second, start the talker:
```bash
docker run -it --rm osrf/ros:galactic-desktop \
  ros2 run demo_nodes_cpp talker
```

This Just Works, and should look like this. The listener:
```bash
kent@ubutower:~$ docker run -it --rm osrf/ros:galactic-desktop   ros2 run demo_nodes_cpp listener
[INFO] [1635348232.126458204] [listener]: I heard: [Hello World: 1]
[INFO] [1635348233.125941180] [listener]: I heard: [Hello World: 2]
[INFO] [1635348234.126072928] [listener]: I heard: [Hello World: 3]
```

The talker:
```bash
kent@ubutower:~$ docker run -it --rm osrf/ros:galactic-desktop   ros2 run demo_nodes_cpp talker
[INFO] [1635348239.684760168] [talker]: Publishing: 'Hello World: 1'
[INFO] [1635348240.684703764] [talker]: Publishing: 'Hello World: 2'
[INFO] [1635348241.684652908] [talker]: Publishing: 'Hello World: 3'
```
Since no network was specified, Docker used the default bridge network **docker0** for communication between containers.

### Nodes on the same Docker container
Start on one terminal a named container running a listener:
```bash
docker run -it --rm --name ros2_demo osrf/ros:galactic-desktop  \
  ros2 run demo_nodes_cpp listener
```

On a second terminal, start a talker running in the same container:
```bash
docker exec -it ros2_demo \
  bash -c "source /opt/ros/galactic/setup.bash && ros2 run demo_nodes_cpp talker"
```
Again Just Works.

## Simple Configurations that Work With Tweaks

So far, we've just used defaults of everything and it has worked. Now we'll explore things that need some simple tweaks to make work.

### One Node on Host, One on Docker

Run the listener again in a Docker container on one terminal:
```bash
docker run -it --rm osrf/ros:galactic-desktop  \
  ros2 run demo_nodes_cpp listener
```

Run the talker on a separate terminal on the host. The listener hears nothing. The talker output looks like this:
```bash
kent@ubutower:~$ ros2 run demo_nodes_cpp talker
1635349269.151703 [0]     talker: using network interface eno1 (udp/192.168.0.59) selected arbitrarily from: eno1, docker0
[INFO] [1635349270.162034418] [talker]: Publishing: 'Hello World: 1'
[INFO] [1635349271.162016336] [talker]: Publishing: 'Hello World: 2'
[INFO] [1635349272.162012236] [talker]: Publishing: 'Hello World: 3'
```
The talker startup warning message tells us the issue - DDS on the talker is using the local ethernet connection's IP address, but the Docker container uses a private Docker address for the default network bridge.

A frequent suggestion on the [ros question and answer site](https://answers.ros.org/questions/) is to use the Docker **host** network option for the Docker container. So repeat the above, but for the listener run instead:
```bash
docker run -it --rm --net host osrf/ros:galactic-desktop  \
  ros2 run demo_nodes_cpp listener
```
This now works.

Discussions on the ROS answer site say that sometimes DDS can try to use shared memory transport with ```--net=host```, which fails. The additional ```--pid=host``` option has been proposed to fix this, see [ROS2 connectivity across Docker containers via Host Driver.](https://answers.ros.org/question/296828/ros2-connectivity-across-docker-containers-via-host-driver/?answer=298320#post-id-298320) I have not experienced the reported failures, but nevertheless in future examples I'll be adding the ```--pid=host``` option whenever I use ```-net=host``` with Docker.

### No "--net host" Parameter Using DDS Driver Configuration

As our network gets more complex, particularly once we add a gateway to cloud ROS2 nodes, we may not want to always the ```--net host``` option. So let's show how to stop using it on a single host.

We need to tell the DDS driver to use the correct interface and IP address in its communications, similarly to the way that we sometimes had to set **ROS_IP** in ROS1. Unfortunately there is no native ROS2 way to do this, it depends on the DDS vendor configuration.

For the ROS2 Galactic version which is used in this demo, the default DDS provider is Cyclone DDS. They briefly give configuration instructions [here](https://github.com/eclipse-cyclonedds/cyclonedds#configuration).

When we start a Docker container without a ```--net``` parameter, we are using the default Docker bridge. From the Linux networking perspective, the host is communicating over a Linux bridge called **docker0** which you can see using ```ip a```. To use this default bridge with Cyclone DDS, we need to set an environment variable **CYCLONEDDS_URI** that specifies to Cyclone DDS which interface to use. That variable can either contain the text of the configuration directly, or a URL to that configuration.

To demo the file option, prepare a configuration file that specifies that the bridge interface **docker0** will be used for DDS communication. Create a file **cyclonedds-docker0.xml** with this content:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>docker0</NetworkInterfaceAddress>
        </General>
    </Domain>
  </CycloneDDS>
```
Tell Cyclone DDS to use that file with this environment variable:
```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds-docker0.xml
```
Run the talker from the same terminal where you did the **export**:
```bash
ros2 run demo_nodes_cpp talker
```
and run the listener in Docker in a separate terminal:
```bash
docker run -it --rm osrf/ros:galactic-desktop  \
  ros2 run demo_nodes_cpp listener
```
The listener should now hear the talker.

If you wanted to run Docker nodes like this generally, you'd want to add the **CYCLONEDDS_URI** variable to your **.bashrc** file for general use (with **$PWD** replaced with the full path to the file). But some caveats. When you are not running any Docker containers, the **docker0** bridge status is set to DOWN. I tried running two host ROS2 nodes with that configuration enabled but no containers so the interface was DOWN, and the communication still worked. Perhaps it was using direct host memory transfer. Worse, if Docker Desktop is not started, then the **docker0** bridge does not exist, and then any ROS2 nodes on the host will not run at all. So you might not want to source CYCLONEDDS_URI, at least until later when we propose some addition changes.

An alternative to the file which works well for simple DDS configurations is to set the text of the configuration directly in **CYCLONEDDS_URI**. That is, instead of setting it to point to the file, define the variable as:
```bash
export CYCLONEDDS_URI='<General><NetworkInterfaceAddress>docker0</></>'
```
Run the example as before with this value and it should also work.

### Docker Containers on Separate Hosts

Get two computers with Docker installed on the same physical network. The second system is supposed to simulate a robot. Try running a Docker talker on one and a Docker listener on the other, without specifying ```--net=host```. We don't expect that to work, and it does not. But it will work if both use the ```--net=host --pid=host``` option. That is, on one system run:
```bash
docker run -it --rm --net=host --pid=host osrf/ros:galactic-desktop ros2 run demo_nodes_cpp talker
```
On the other,
```bash
docker run -it --rm --net=host --pid=host osrf/ros:galactic-desktop ros2 run demo_nodes_cpp listener
```
This should work.

There are ways to make the cross-host Docker nodes work without ```--net=host``` but things start getting more complex. (Hint: move the network connector as an interface under the Docker bridge, then give the Docker bridge an IP address on the local network.) 

In the next installment of this series I'll move to the OpenVPN configuration which we will use with the cloud connections.
