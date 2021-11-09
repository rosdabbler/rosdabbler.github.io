---
title: "Experiments reusing the docker macvlan network"
layout: archive
author_profile: true
tags: [docker, networking]
date: 2021-10-23
---

Macvlan networks are supposed to be simpler alternatives to bridges. Let's try that with ROS2.

Usually macvlan networks are discussed in the context of wanting to use the addressing scheme from the underlying network. Let met start there, and see if that works.

## Macvlan Access to Local Network
My local network is 192.168.0.0/24 with DHCP providing *.40 - *.100. The gateway is 192.168.0.11, the local ethernet adapter is eno1.

Create a docker macvlan:
```bash
docker network create -d macvlan \
  --subnet=192.168.0.0/24 \
  --gateway=192.168.0.11 \
  --ip-range=192.168.0.128/26 \
  -o macvlan_mode=bridge \
  -o parent=eno1 \
rosmac
```
Run a basic container with this:
```bash
docker run -it --rm --net=rosmac busybox
```

From that container, I can successfully ping the gateway (192.168.0.11), another machine on the network (192.168.0.210), and a Google DNS server (8.8.8.8). I cannot ping the host (192.168.0.59). This is a known restriction, with a [known workaround](https://blog.oddbit.com/post/2018-03-12-using-docker-macvlan-networks/): add a macvlan network on the host.

```bash
sudo ip link add local-ip link eno1 type macvlan mode bridge
sudo ip addr add 192.168.0.211/32 dev local-ip
sudo ip link set local-ip up
sudo ip route add 192.168.0.128/26 dev local-ip
```

With that fix, I can ping the host (192.168.0.59) from the busybox container.

## ROS2 tests with Macvlan on Local Network

### Two containers
Run a listener in one container:
```bash
docker run -it --rm --net=rosmac osrf/ros:galactic-desktop \
  ros2 run demo_nodes_cpp listener
```

From a separate terminal, run a talker in another container:
```bash
docker run -it --rm --net=rosmac osrf/ros:galactic-desktop \
  ros2 run demo_nodes_cpp talker
```

This works, the listener hears the talker.

### One node in container, one in host.

Run the listener as before. Run the talker on the host:
```bash
ros2 run demo_nodes_cpp talker
```

This does **NOT** work. The talker reports:
```
1635377271.570346 [0]     talker: using network interface eno1 (udp/192.168.0.59) selected arbitrarily from: eno1, docker0, local-ip
```
so the multicast discovery packets are not being sent to the container. Try this config file:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>local-ip</NetworkInterfaceAddress>
        </General>
    </Domain>
  </CycloneDDS>
```
saved as ```cyclonedds-local-ip.xml```. Use this file when starting the host talker:
```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds-local-ip.xml
ros2 run demo_nodes_cpp talker
```
With that change, the listener hears the talker.

I also tried on a separate system:
```
ros2 run demo_nodes_cpp talker
```
and this also worked just fine.

Does this setting interfere with two nodes on the host? Kill the Docker container, and run the listener as:
```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds-local-ip.xml
ros2 run demo_nodes_cpp listener
```
That also seems to work. It appears that a macvlan network is a viable alternative to ```--net=host```

## Macvlan with different IP subnet

I removed the previously created macvlan networks:
```bash
sudo ip link del local-ip
docker network rm rosmac
```

I recreated them with IP addresses that differed from the underlying network.
```
docker network create -d macvlan \
  --subnet=10.231.96.0/20 \
  --ip-range=10.231.101.0/24 \
  -o macvlan_mode=bridge \
  -o parent=eno1 \
rosmac
#
sudo ip link add local-ip link eno1 type macvlan mode bridge
sudo ip addr add 10.231.100.0/32 dev local-ip
sudo ip link set local-ip up
sudo ip route add 10.231.101.0/24 dev local-ip
```
Test with busybox:
```bash
docker run -it --rm --net=rosmac busybox
```
With that, I can ping the host at 10.231.100.0, but nothing else really works. ```ping 8.8.8.8``` fails. Not surprising, NAT is done by the router, so subnet 10.231.96.0/20 is not returning to the host. Perhaps I could use NAT locally for packets from the 10.231.101.0/24 subnet.

At this point, what am I trying to accomplish? I want to allow alternate IP addresses on the ROS2 robot subnet from the local network, but minimize the setup, and also try to let the non-ROS links be as little affected as possible. The competition is probably dual-homing the ethernet connection. And this really only makes sense on the robot itself. The desktop and cloud gateways are going to need a bridge to decide where to send packets. So let me concentrate on the gateway configuration, assuming that the robot can work with a dual IP address network.