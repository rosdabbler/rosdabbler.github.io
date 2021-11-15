---
title: "Netplan Apply Failure with tap0 and eno1"
layout: rkjnote
author_profile: true
tags: [openvpn, networking]
date: 2021-11-09
---
While testing [Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway](/adventures-in-ros2-networking-4) I came across a case where `sudo netplan apply` failed to add ethernet to the bridge. Let's see if I can reproduce that.

Remove **/etc/netplan/90-rosbridge.yaml** from the configuration be renaming it with a ".disabled" suffix. `sudo netplan apply`. No more bridge, **en01** has an IP address. Get OpenVPN running on the Cloud Gateway, and connect using the config file from [Adventures in ROS2 Networking 2 - OpenVPN gateway ](/adventures-in-ros2-networking-2) `sudo openvpn ros_desktop.ovpn`.

Restore again the proper name to **/etc/netplan/90-rosbridge.yaml** then `sudo netplan apply`.
```bash
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
    inet 192.168.0.55/24 brd 192.168.0.255 scope global dynamic noprefixroute eno1
       valid_lft 7198sec preferred_lft 7198sec
    inet6 fe80::4504:135b:d393:8ed3/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 100
    link/ether 0e:ae:19:fc:61:b7 brd ff:ff:ff:ff:ff:ff
    inet 192.168.255.2/24 brd 192.168.255.255 scope global tap0
       valid_lft forever preferred_lft forever
    inet6 fe80::cae:19ff:fefc:61b7/64 scope link 
       valid_lft forever preferred_lft forever
6: rosbridge: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000
    link/ether fe:25:4c:50:fb:5c brd ff:ff:ff:ff:ff:ff
```
That is, **rosbridge** has no IP, **eno1** is not under the bridge. Try:
```bash
sudo ip link set down eno1
sleep 5
sudo netplan apply
```
That fixes it.

Reproduce again to the point that `sudo netplan apply` fails. Now try disconnecting from OpenVPN. tap0 disappears, but problem persists. Moral of the story seems to be to take eno1 down prior to `sudo netplan apply`. I recall having similar issues trying to change the ip address of **eno1**. A reboot also fixes it.

I'm not sure what the ramifications of this are. It mostly appears in the middle of config testing.