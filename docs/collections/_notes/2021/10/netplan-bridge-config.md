---
title: "Netplan Bridge Configuration"
layout: rkjnote
author_profile: true
tags: [networking]
date: 2021-10-31
---
From what I have learned previously, openVPN gateway computers will need to have their layer 2 connectivity going through a bridge, which will contain both the OpenVPN tunnel tap0, as well as the local network and Docker connections. This post deals with how to configure that bridge.

## Introduction

I've learned in this process a lot of what makes Linux networking tick, at least for the Ubuntu systems that I use. Generally, there are various categories of programs that operate at different levels.

One level of program operates immediately on the state of the system, but does not persist those changes after reboot. Most networking examples show these commands, but they are not really what you need, since in most cases you need to persist the changes. At least in modern Ubuntu, the **ip** command is the do-all version of this. There are others that could be used as well, but you really need to master **ip** if you are going to be doing non-standard Linux networking.

At the next level are programs that maintain a process data files of desired connections, and restore these after reboot (or perhaps also when an event happens, like an ethernet cable is plugged in). There are two main variants of these with completely different configuration files - **NetworkManager** and **networkd**. Desktop Ubuntu seems to use **NetworkManager**, but the AWS images use **networkd**.

At the next level, there is a program called **netplan** that can work with either **NetworkManager** or **networkd**. This works by readings a set of its own configuration files, and then generating (or modifying) the **NetworkManager** or **networkd** transient files in **/run/**. For our purposes, we will work mostly with **netplan** configurations so that our designs will work with either either **NetworkManager** or **networkd**. You can see a reference manual for **netplan** [here](https://netplan.io/reference/).

We are going to create a Linux bridge with the name **rosbridge** that will handle the OpenVPN traffic. A bridge is like a network switch, and individual real or virtual ethernet cards are plugged into it (called **links** in Linux networking language). It learns by listening which device with a particular mac address is found on which link, so it sort of serves as an ethernet router.

Our ROS2 network needs to resemble a fully-connected local ethernet network for the DDS node discovery process to work. For that reason, the main device that any machine uses to send ROS2 traffic needs to be the bridge, as it can then properly route the traffic to the correct link, which could either be an OpenVPN-managed **tap** layer 2 tunnel, a local ethernet or wifi adapter, or a virtual ethernet adapter connected to a Docker container.

The point of this post is how to configure that bridge using **netplan**.

## Create the Bridge

Files in **/etc/netplan** will be processed in lexigraphical order, so following convention we will name our config file with a high number so it is processed late (and can override any earlier files). The config file follows **yaml** format, and must have a **.yaml** suffix. Add this content to a file **/etc/netplan/90-rosbridge.yaml**:
```
network:
  version: 2
  bridges:
    rosbridge:
      mtu: 1500
      addresses: [10.1.2.3/24]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
```
Then apply the file with **sudo netplan apply** and show the effect with **ip a**
```bash
kent@ubutower:/etc/netplan$ sudo netplan apply
kent@ubutower:/etc/netplan$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eno1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether d0:17:c2:99:00:a4 brd ff:ff:ff:ff:ff:ff
    altname enp0s25
    inet 192.168.0.59/24 brd 192.168.0.255 scope global noprefixroute eno1
       valid_lft forever preferred_lft forever
    inet6 fe80::4504:135b:d393:8ed3/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
18: rosbridge: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000
    link/ether 7a:dd:49:70:14:61 brd ff:ff:ff:ff:ff:ff
    inet 10.1.2.3/24 brd 10.1.2.255 scope global noprefixroute rosbridge
       valid_lft forever preferred_lft forever
```
That created the bridge, but it is currently not connected to anything and is down. What we need to do is to add our internet address as n interface under the bridge. That is, we are "plugging in" the ethernet connection into the "switch" which is the bridge. I've also enabled dhcp so that the bridge will get an IP address from the network instead of specifying it. Change /etc/netplan/90-rosbridge.yaml to:
```
network:
  version: 2
  ethernets:
    eno1:
      match:
        name: "eno1"

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [eno1]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
Apply netplan, and test:
```bash
kent@ubutower:/etc/netplan$ sudo netplan apply
kent@ubutower:/etc/netplan$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eno1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UP group default qlen 1000
    link/ether d0:17:c2:99:00:a4 brd ff:ff:ff:ff:ff:ff
    altname enp0s25
19: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether d0:17:c2:99:00:a4 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.55/24 brd 192.168.0.255 scope global dynamic noprefixroute rosbridge
       valid_lft 7199sec preferred_lft 7199sec
    inet6 fe80::d217:c2ff:fe99:a4/64 scope link 
       valid_lft forever preferred_lft forever
kent@ubutower:/etc/netplan$ ping 8.8.8.8
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=116 time=2.57 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=116 time=2.74 ms
```

The ethernet adapter description now has the phrase **master rosbridge** which is how you know that it is connected to the bridge. With this change, the 
link that the system sees for you main connection to the internet is no longer your ethernet adapter, but instead is the bridge:
```
kent@ubutower:/etc/netplan$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    425    0        0 rosbridge
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 eno1
192.168.0.0     0.0.0.0         255.255.255.0   U     425    0        0 rosbridge
```
Note that this configuration takes over the configuration of the internet card from the underlying system like NetworkManager, so any changes you do in the NetworkManager user interface (that is, in Ubuntu settings) will not have any effect.

Different systems will have different names for the ethernet adapter. We can match any ethernet adapter for a more general version of this file like this:
```
network:
  version: 2
  ethernets:
    bridge-me:
      match:
        name: "*"

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [bridge-me]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
This gives the same results as before, but will work regardless of the name of the ethernet adapter.

I actually use a static IP address for my main desktop syste, so the file I actually use, with a static address, is:
```
network:
  version: 2
  ethernets:
    bridge-me:
      match:
        name: "*"

  bridges:
    rosbridge:
      mtu: 1500
      addresses: [192.168.0.59/24]
      interfaces: [bridge-me]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
```