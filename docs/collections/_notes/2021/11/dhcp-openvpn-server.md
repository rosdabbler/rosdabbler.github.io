---
title: "DHCP with an OpenVPN server bridge"
layout: rkjnote
author_profile: true
tags: [openvpn]
date: 2021-11-01
---
To extend a simple local robot network using a local DHCP server, I would like for the OpenVPN server bridge to get its IP address from the local DHCP server. How to do that?

I played some with manual commands, got it to work, but rebooting the virtual machine killed it. Trying again.

Using basic install file ain4-install.sh

Here's a **/etc/netplan/90-rosbridge.yaml** file that adds the tap0 interface to the bridge:
```
network:
  version: 2
  ethernets:
    tap0:
      match:
        name: "tap0"

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [tap0]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
I created this file, then
```bash
sudo netplan apply
```
Parsed OK, no tap0 created. As I expected. Now configure and start openvpn.
```bash
sudo ovpn_genconfig -u udp://$OPENVPN_URL -t -d -D
sudo ovpn_initpki
sudo easyrsa build-client-full ros_local_gateway nopass
sudo ovpn_getclient ros_local_gateway > ros_local_gateway.ovpn
sudo systemctl start openvpn@openvpn
```
With that, I get:
```bash
ubuntu@ip-172-31-24-240:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:fe:c5:16:53:e9 brd ff:ff:ff:ff:ff:ff
    inet 172.31.24.240/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 3154sec preferred_lft 3154sec
    inet6 fe80::fe:c5ff:fe16:53e9/64 scope link 
       valid_lft forever preferred_lft forever
3: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:ee:09:7f:1d brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
4: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 5e:45:25:54:f2:eb brd ff:ff:ff:ff:ff:ff
    inet6 fe80::d8e2:8fff:fe78:236e/64 scope link 
       valid_lft forever preferred_lft forever
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether 5e:45:25:54:f2:eb brd ff:ff:ff:ff:ff:ff
```
Things to note:
- tap0 was joined to rosbridge
- rosbridge did not get an ethernet address

Will this still work with openvpn?

I started openvpn from the client. Server rosbridge did not get an IP address, but tap0 stayed with the bridge. On server, routes are:
```bash
root@ip-172-31-24-240:/etc/netplan# route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.16.1     0.0.0.0         UG    100    0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.31.16.0     0.0.0.0         255.255.240.0   U     0      0        0 eth0
172.31.16.1     0.0.0.0         255.255.255.255 UH    100    0        0 eth0
```
so no tap0 routes applies, which makes sense with no ip address. **netplan apply** does not change it. Back to the client, tap0 is not under the bridge. Fix that with **sudo brctl addif rosbridge tap0**. Fixed client, no change on server.

Tried **netplan apply** on server. No help. But this worked:
```bash
root@ip-172-31-24-240:/etc/netplan# ip link set down rosbridge
root@ip-172-31-24-240:/etc/netplan# ip link set up rosbridge
root@ip-172-31-24-240:/etc/netplan# ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:fe:c5:16:53:e9 brd ff:ff:ff:ff:ff:ff
    inet 172.31.24.240/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 3557sec preferred_lft 3557sec
    inet6 fe80::fe:c5ff:fe16:53e9/64 scope link 
       valid_lft forever preferred_lft forever
3: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:ee:09:7f:1d brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
4: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 5e:45:25:54:f2:eb brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.97/24 brd 192.168.0.255 scope global dynamic rosbridge
       valid_lft 7199sec preferred_lft 7199sec
    inet6 fe80::5c45:25ff:fe54:f2eb/64 scope link 
       valid_lft forever preferred_lft forever
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether 5e:45:25:54:f2:eb brd ff:ff:ff:ff:ff:ff
```
Routing tables are messed up. On client:
```bash
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    425    0        0 rosbridge
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 eno1
192.168.0.0     0.0.0.0         255.255.255.0   U     425    0        0 rosbridge
192.168.255.0   0.0.0.0         255.255.255.0   U     0      0        0 tap0
```
On server:
```bash
ubuntu@ip-172-31-24-240:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.16.1     0.0.0.0         UG    100    0        0 eth0
0.0.0.0         192.168.0.11    0.0.0.0         UG    100    0        0 rosbridge
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.31.16.0     0.0.0.0         255.255.240.0   U     0      0        0 eth0
172.31.16.1     0.0.0.0         255.255.255.255 UH    100    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     0      0        0 rosbridge
192.168.0.11    0.0.0.0         255.255.255.255 UH    100    0        0 rosbridge
```
From client, **ping 192.168.0.97** sometimes worked, sometimes not. Fixed routes on server like this, but I really need to keep them from being created in the first place:
```bash
ubuntu@ip-172-31-24-240:~$ sudo ip route del default dev rosbridge
ubuntu@ip-172-31-24-240:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.16.1     0.0.0.0         UG    100    0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
172.31.16.0     0.0.0.0         255.255.240.0   U     0      0        0 eth0
172.31.16.1     0.0.0.0         255.255.255.255 UH    100    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     0      0        0 rosbridge
192.168.0.11    0.0.0.0         255.255.255.255 UH    100    0        0 rosbridge
```
So this kinda works, but I need to automate the manual steps of adding client tap0 to bridge, and stopping default route through rosbridge on server.
