---
title: "Experiments replacing eth0 with a bridge on aws"
layout: rkjnote
author_profile: true
tags: [aws, networking]
date: 2021-10-28
---
The goal here is to replace the main ethernet connector with a bridge, so that I will be able to insert the tap interface of openvpn, and then use the gateway from other machines in the AWS virtual private cloud subnet. I've already established that I can put files in /run/netplan that go away on reboot, but are seen by **sudo netplan apply** for testing.

My aws machine has this file /etc/netplan/50-cloud-init.yaml:
```
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    ethernets:
        eth0:
            dhcp4: true
            dhcp6: false
            match:
                macaddress: 0a:30:2b:2d:ca:8d
            set-name: eth0
    version: 2
```

I'll copy this to /run/netplan/90-test.yaml and then modify. First copy no mods, **sudo netplan --debug apply**, and I still have connectivity. Next add a bridge:
```
network:
    ethernets:
        eth0:
            dhcp4: true
            dhcp6: false
            match:
                macaddress: 0a:30:2b:2d:ca:8d
            set-name: eth0
    version: 2
    bridges:
        rosbridge:
            addresses: [10.231.161.1/20]
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: false
            dhcp6: false
```
That worked. Next, I will try to add the ethernet port to the bridge, using the example fromhttps://netplan.io/reference/.
First, let me test adding items to the bridge. Can I add another ethernet card?
```
network:
    version: 2
    ethernets:
        switchports:
            match: {name: "eth1"}
        eth0:
            dhcp4: true
            dhcp6: false
            match:
                macaddress: 0a:30:2b:2d:ca:8d
            set-name: eth0
        eth1:
            macaddress: 01:02:03:04:05:06
            match:
                macaddress: 01:02:03:04:05:06
            set-name: eth1
    bridges:
        rosbridge:
            interfaces: [switchports]
            addresses: [10.231.161.1/20]
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: false
            dhcp6: false
```
That did not work, **netplan apply** worked, but no **eth1** in **ip a**. I've been using my working openvpn machine, at this point I want to experiment more, so I'll create a new machine. ... now the default netplan is:
```
network:
    ethernets:
        eth0:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 100
            dhcp6: false
            match:
                macaddress: 02:ff:8c:d5:6b:69
            set-name: eth0
        eth1:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 200
            dhcp6: false
            match:
                macaddress: 02:fe:b5:ce:53:c7
            set-name: eth1
    version: 2
```
I'll try to add eth1 to the bridge:
```
network:
    version: 2
    ethernets:
        switchports:
            match: {name: "eth1"}
        eth0:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 100
            dhcp6: false
            match:
                macaddress: 02:ff:8c:d5:6b:69
            set-name: eth0
        eth1:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 200
            dhcp6: false
            match:
                macaddress: 02:fe:b5:ce:53:c7
            set-name: eth1
    bridges:
        rosbridge:
            interfaces: [switchports]
            addresses: [10.231.161.1/20]
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: false
            dhcp6: false
```
Applied, but eth1 is not in the bridge. Docs say you may need to match by macaddress,let be try that. Tried, no luck. Maybe just use the interface name? OK that worked:
```
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 100
            dhcp6: false
            match:
                macaddress: 02:ff:8c:d5:6b:69
            set-name: eth0
        eth1:
            dhcp4: true
            dhcp4-overrides:
                route-metric: 200
            dhcp6: false
            match:
                macaddress: 02:fe:b5:ce:53:c7
            set-name: eth1
    bridges:
        rosbridge:
            interfaces: [eth1]
            addresses: [10.231.161.1/20]
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: false
            dhcp6: false
```
The eth1 link. Note DHCP did not work there.
```bash
root@ip-10-231-171-242:/run/netplan# ip link show eth1
3: eth1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UP mode DEFAULT group default qlen 1000
    link/ether 02:fe:b5:ce:53:c7 brd ff:ff:ff:ff:ff:ff
```
OK, now the real fun. I'm going to move the macaddress of eth0 to the bridge, and see if DHCP will work there. I expect this will fail.
```
network:
    version: 2
    ethernets:
        eth1:
            dhcp4: false
            dhcp4-overrides:
                route-metric: 200
            dhcp6: false
            match:
                macaddress: 02:fe:b5:ce:53:c7
            set-name: eth1
    bridges:
        rosbridge:
            interfaces: [eth1]
            macaddress: 02:ff:8c:d5:6b:69
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: true
            dhcp6: false
```
Yep, that killed connectivity. Reboot from the web interface.

My hope was to use the AWS network to support multicast packets to use with ROS2, with a bridge on a gateway with a tap0 additional interface to support communication with devices over OpenVPN. If I believe [this blog](https://yurmagccie.wordpress.com/2019/08/13/aws-networking-part-2-bridging-and-routing/) then:
> By default, every Elastic Network Interface (ENI) has Source/Destination Check attribute enabled. It means that every packet that leaves/enters this ENI will be checked if IP source/destination addresses of the packet match to AWS VPC/EC2 configuration.

I checked, and yes there is that option on the AWS network interface definition. But also,

> AWS networking differs from conventional on-perm networks. For example, it does not support broadcast or multicast frames

But following links there, apparently AWS now has something called "AWS Transit Gateway" which among other things, 

[With native support for multicast, AWS Transit Gateway enables customers to easily deploy their multicast applications in the cloud](https://aws.amazon.com/about-aws/whats-new/2019/12/run-ip-multicast-workloads-aws-transit-gateway/)

So I'm confused. If the *AWS Transit Gateway* allows multicast and connections between VPCs, the surely the VPCs must support multicast? But also OpenVPN does not really handle multicast correctly, I understand it converts them to broadcasts instead. But none of this matters if I can't get a bridge running on AWS. Let me try disabling strict source checking on the network interfaces, and use the previous 90-test.yaml

Hmmm, that worked but eth0 still exists as well as the bridge, and the default route goes through it. rosbridge did not get an IP address from DHCP.

Maybe I should see if AWS has any hints. Looking at their robot stuff, I see https://aws.amazon.com/blogs/robotics/deploy-and-manage-ros-robots-with-aws-iot-greengrass-2-0-and-docker/ Looks like what they do is convert ROS2 messages into mqtt, then run those on AWS with mqtt communication.

But onward. I can add the eth1 address to the rosbridge using **ip a add 10.231.163.206/20 dev rosbridge** But how can I test it? I need another instance in the same subnet.

OK I can ping the new instance from the old (after enabling a security group that supports ping). `route -n` shows both rosbridge and eth0 routing 10.231.160.0/20. If I delete the eth0 route, then I lose ping to the other instance, but not internet connectivity. The bridge is not working. Probably expected given its MAC address.

Let me try the whole thing again, now without using the eth0 mach address, but manually using the eth1 address.
```
network:
    version: 2
    ethernets:
        eth1:
            dhcp4: false
            dhcp4-overrides:
                route-metric: 200
            dhcp6: false
            match:
                macaddress: 02:fe:b5:ce:53:c7
            set-name: eth1
    bridges:
        rosbridge:
            interfaces: [eth1]
            addresses: [10.231.163.206/20]
            mtu: 1500
            nameservers:
                addresses: [8.8.8.8, 8.8.4.4]
            parameters:
                stp: false
            dhcp4: true
            dhcp6: false
```
`netplan --debug apply` I have:
```bash
root@ip-10-231-171-242:/run/netplan# ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:ff:8c:d5:6b:69 brd ff:ff:ff:ff:ff:ff
    inet 10.231.171.242/20 brd 10.231.175.255 scope global dynamic eth0
       valid_lft 3580sec preferred_lft 3580sec
    inet6 fe80::ff:8cff:fed5:6b69/64 scope link 
       valid_lft forever preferred_lft forever
3: eth1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UP group default qlen 1000
    link/ether 02:fe:b5:ce:53:c7 brd ff:ff:ff:ff:ff:ff
4: docker0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
    link/ether 02:42:ea:68:c1:ee brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
       valid_lft forever preferred_lft forever
5: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 02:ff:8c:d5:6b:69 brd ff:ff:ff:ff:ff:ff
    inet 10.231.163.206/20 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::ff:8cff:fed5:6b69/64 scope link 
       valid_lft forever preferred_lft forever
root@ip-10-231-171-242:/run/netplan# route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         10.231.160.1    0.0.0.0         UG    100    0        0 eth0
10.231.160.0    0.0.0.0         255.255.240.0   U     0      0        0 eth0
10.231.160.1    0.0.0.0         255.255.255.255 UH    100    0        0 eth0
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
root@ip-10-231-171-242:/run/netplan# ping 10.231.165.49
PING 10.231.165.49 (10.231.165.49) 56(84) bytes of data.
64 bytes from 10.231.165.49: icmp_seq=1 ttl=64 time=0.542 ms
64 bytes from 10.231.165.49: icmp_seq=2 ttl=64 time=0.509 ms
```
Now `ip route change 10.231.160.0/20 dev rosbridge` and `ping 10.231.165.49` fails. I've pretty much failed to replace an ethernet connection with a bridge.

I think the next theing I should try is going to be a local tunnel, but I need to get that working on my local computers first.

**terminate these test instances**
