---
title: "vxlan with ROS2 in local network"
layout: rkjnote
author_profile: true
tags: [networking]
date: 2021-10-24
---
I want to experiment with setting up a vxlan in my local network, and see if ROS2 will work ver it.

## vxlan
I have two machines. One is at 192.168.0.59, the other at 192.168.0.210. Both are Ubuntu 20.04 installs. Follwing roughly https://programmer.help/blogs/practice-vxlan-under-linux.html, setup a vxlan link between them.

On .59 machine:
```bash
sudo su
ip link add vxlan1 type vxlan remote 192.168.0.210 local 192.168.0.59 dev eno1 id 100 dstport 4789
ip a add 10.0.0.1/24 dev vxlan1
ip link set up vxlan1
```

On .210 machine:
```bash
ip link add vxlan1 type vxlan remote 192.168.0.59 local 192.168.0.210 dev eno1 id 100 dstport 4789
ip a add 10.0.0.2/24 dev vxlan1
ip link set up vxlan1
```

Can I ping? On .59:
```bash
root@ubutower:/home/kent# ping 10.0.0.2
PING 10.0.0.2 (10.0.0.2) 56(84) bytes of data.
64 bytes from 10.0.0.2: icmp_seq=1 ttl=64 time=0.632 ms
64 bytes from 10.0.0.2: icmp_seq=2 ttl=64 time=0.609 ms
```
Yes! How about ROS2? On each setup a DDS config file vxlan1.xml:
```
<?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>vxlan1</NetworkInterfaceAddress>
        </General>
    </Domain>
  </CycloneDDS>
```
On .59:
```bash 
CYCLONEDDS_URI=file://$PWD/vxlan1.xml ros2 run demo_nodes_cpp listener
```
on .210:
```bash
CYCLONEDDS_URI=file://$PWD/vxlan1.xml ros2 run demo_nodes_cpp talker
```
It works! The next thing that I need to try is through rosbridge. Here is /etc/netplan/rosbridge.yaml
```
etwork:
  version: 2
  bridges:
    rosbridge:
      addresses: [10.0.0.1/20]
      mtu: 1500
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
```
Configure:
```bash
ip a del 10.0.0.1 dev vxlan1
netplan apply
ip link set vxlan1 master rosbridge
```
On .210 /etc/netplan/rosbridge.yaml:
```
network:
  version: 2
  bridges:
    rosbridge:
      addresses: [10.0.0.2/20]
      mtu: 1500
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
```
Configure:
```bash
ip a del 10.0.0.2/24 dev vxlan1
netplan apply
ip link set vxlan1 master rosbridge
ip link set up vxlan1
```
I tried a ping, it failed. I suspect the addresses are incorrect. Let me try wireshark. The ping sends an ARP for 10.0.0.2 to rosbridge, which also shows on vxlan1, but no response is ever received.

OK looking at .210, I cannot set rosbridge up. OK, vxlan1 does not have master set correctly. Doing that, I can now ping! Now I just need to understand the LOCAL and REMOTE meanings.
