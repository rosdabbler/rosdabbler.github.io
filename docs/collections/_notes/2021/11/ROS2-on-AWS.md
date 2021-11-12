---
title: "ROS2 nodes on multiple AWS machines"
layout: archive
author_profile: true
tags: [networking, aws]
date: 2021-11-09
---
I need to see if ROS2 works between machines on an AWS Virtual Private Cloud network.

## Initial setup
I created two AWS EC2 virtual machines. I install each according the beginnings of [https://rosdabbler.github.io/rosdabbler.github.io/adventures-in-ros2-networking-2][/adventures-in-ros2-networking-2] but I do not configure OpenVPN. That is,
```bash
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain2-install.sh
source ain2-install.sh
```
I'll give these the hostnames **aws1** and **aws2** using eg `sudo hostnamectl set-hostname aws1`
I exit ssh and connect again so that the hostname shows up in bash.

Try a basic talker and listener. aws1:
```bash
ubuntu@aws1:~$ ros2 run demo_nodes_cpp talker
[INFO] [1636515713.282799491] [talker]: Publishing: 'Hello World: 1'
[INFO] [1636515714.282791276] [talker]: Publishing: 'Hello World: 2'
```
aws2:
```bash
ubuntu@aws2:~$ ros2 run demo_nodes_cpp listener
```
In other words, crickets (doesn't work). Yet I can ping:
```bash
ubuntu@aws2:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:0d:62:fb:de:df brd ff:ff:ff:ff:ff:ff
    inet 172.31.23.171/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 2564sec preferred_lft 2564sec
    inet6 fe80::d:62ff:fefb:dedf/64 scope link 
       valid_lft forever preferred_lft forever
```
```bash
ubuntu@aws1:~$ ping 172.31.23.171
PING 172.31.23.171 (172.31.23.171) 56(84) bytes of data.
64 bytes from 172.31.23.171: icmp_seq=1 ttl=64 time=1.53 ms
64 bytes from 172.31.23.171: icmp_seq=2 ttl=64 time=0.498 ms
```
[Previously](/notes/2021/10/local-vxlan/ "vxlan with ROS2 in local network") I did some experiments using a VXLAN to connect two ROS2 segments in a local netowork. Let me try that in AWS. Let's assume that aws1 is the gateway, and aws2 an additional cloud processor.

But first, let me try to loosen security rules for cloud connections.
Going to AWS EC2 console, I:
- copy the existing security group to a new one.
- add all traffic from the local VPC subnet (172.31.16.0/20)

I end up with the following security group:
![vxlan security group](/assets/images/allVPC-security.png "AWS configuration of all-VPC-allowed security group")

I apply this group to both running instances in the AWS EC2 console. Check ping, still works. Try talker/listener. Does not work. Reboot both instances for laughs. Talker/listener still does not work.

AWS has [info](https://docs.aws.amazon.com/vpc/latest/tgw/manage-domain.html "Managing multicast domains") that implies multicast can work. It always seems to involve a "transit gateway".

Also, looking at <https://www.packetmischief.ca/2020/02/13/multicast-routing-in-aws/> I see:
> Even if the sender and all of the receivers are in the same VPC, even if they're in the same subnet in the VPC, there is still a need for a multicast-enabled TGW and a multicast domain as described below.

I also see from <https://aws.amazon.com/transit-gateway/pricing/> that there are additional charges in AWS associated with a transit gateway, $0.05 per hour (per VPC?) plus traffic charges. That's $36 per month ignoring the traffic charges. Being a cheap non-supported researcher, I'd like to avoid that plus the extra complexity.

There are also configuration options with the DDS providers to change how discovery is done, perhaps we could bypass these issues that way. But I have not gone down that path yet. Also, I'll need a bridge for OpenVPN tap0 to talk to the robot network, and I have failed to make that work.

So what I will try is layer 2 VXLAN tunnels from the separate AWS instances to the central gateway, with those instances added to a bridge on the Cloud Gateway that also contains the tap0 interface for OpenVPN.

I'll start just trying it manually. Note that the aws1 IP is 172.31.22.156/20, aws2 is 172.31.23.171. Avahi local resolution (aws1.local) is not working, so I'll need to use for now raw IP addresses.

As [Previously](/notes/2021/10/local-vxlan/ "vxlan with ROS2 in local network") done in the local network, on aws1:
```bash
root@aws1:/home/ubuntu# ip link add vxlan1 type vxlan remote 172.31.23.171 local 172.31.22.156 dev eth0 id 100 dstport 4789
root@aws1:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@aws1:/home/ubuntu# ip link set up vxlan1
root@aws1:/home/ubuntu# ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:ca:7e:1c:0e:87 brd ff:ff:ff:ff:ff:ff
    inet 172.31.22.156/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 1804sec preferred_lft 1804sec
    inet6 fe80::ca:7eff:fe1c:e87/64 scope link 
       valid_lft forever preferred_lft forever
3: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue state UNKNOWN group default qlen 1000
    link/ether 82:6a:55:ec:d4:47 brd ff:ff:ff:ff:ff:ff
    inet 192.168.255.100/24 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::806a:55ff:feec:d447/64 scope link 
       valid_lft forever preferred_lft forever
```
On aws2:
```bash
ubuntu@aws2:~$ sudo su
root@aws2:/home/ubuntu# ip link add vxlan1 type vxlan remote 172.31.22.156 local 172.31.23.171 dev eth0 id 100 dstport 4789
root@aws2:/home/ubuntu# ip a add 192.168.255.101/24 dev vxlan1
root@aws2:/home/ubuntu# ip link set up vxlan1
root@aws2:/home/ubuntu# ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:0d:62:fb:de:df brd ff:ff:ff:ff:ff:ff
    inet 172.31.23.171/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 3436sec preferred_lft 3436sec
    inet6 fe80::d:62ff:fefb:dedf/64 scope link 
       valid_lft forever preferred_lft forever
3: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue state UNKNOWN group default qlen 1000
    link/ether 7e:6a:e5:45:ec:44 brd ff:ff:ff:ff:ff:ff
    inet 192.168.255.101/24 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::7c6a:e5ff:fe45:ec44/64 scope link 
       valid_lft forever preferred_lft forever
```
(First time I neglected the /24 in the addresses, above corrected to show the correct way to do this)
Now:
```bash
root@aws2:/home/ubuntu# ping 192.168.255.100
PING 192.168.255.100 (192.168.255.100) 56(84) bytes of data.
64 bytes from 192.168.255.100: icmp_seq=1 ttl=64 time=0.798 ms
64 bytes from 192.168.255.100: icmp_seq=2 ttl=64 time=0.438 ms
```
and
```bash
root@aws1:/home/ubuntu# ping 192.168.255.101
PING 192.168.255.101 (192.168.255.101) 56(84) bytes of data.
64 bytes from 192.168.255.101: icmp_seq=1 ttl=64 time=0.660 ms
64 bytes from 192.168.255.101: icmp_seq=2 ttl=64 time=0.569 ms
```
Success, at least for first phase. Now for ROS2. Need to configure for adapter vxlan1, so:
```bash
ubuntu@aws2:~$ CYCLONEDDS_URI="<General><NetworkInterfaceAddress>vxlan1</></>" ros2 run demo_nodes_cpp talker
[INFO] [1636520978.410060881] [talker]: Publishing: 'Hello World: 1'
[INFO] [1636520979.409900238] [talker]: Publishing: 'Hello World: 2'
```
and
```bash
ubuntu@aws1:~$ CYCLONEDDS_URI="<General><NetworkInterfaceAddress>vxlan1</></>" ros2 run demo_nodes_cpp listener
[INFO] [1636520978.401013501] [listener]: I heard: [Hello World: 1]
[INFO] [1636520979.400804518] [listener]: I heard: [Hello World: 2]
```

OK that's cool, it worked! Ultimately at the gateway, vxlan1 will be setup as an adapter under rosbridge. But let me better understand the vxlan setup first, like I don't really understand the meaning of **local** and **remote**.

I reboot both systems to remove any residual configuration.

I tried setup with no remote or local:
```bash
root@aws1:/home/ubuntu# ip link add vxlan1 type vxlan dev eth0 id 100 dstport 4789
root@aws1:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@aws1:/home/ubuntu# ip link set up vxlan1
```
```bash
root@aws2:/home/ubuntu# ip link add vxlan1 type vxlan dev eth0 id 100 dstport 4789
root@aws2:/home/ubuntu# ip a add 192.168.255.101/24 dev vxlan1
root@aws2:/home/ubuntu# ip link set up vxlan1
```
Ping failed. Add back in the local:
```bash
root@aws1:/home/ubuntu# ip link del vxlan1
root@aws1:/home/ubuntu# ip link add vxlan1 type vxlan local 172.31.22.156 dev eth0 id 100 dstport 4789
root@aws1:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@aws1:/home/ubuntu# ip link set up vxlan1
```
```bash
ip lin del vxlan1
root@aws2:/home/ubuntu# ip link add vxlan1 type vxlan local 172.31.23.171 dev eth0 id 100 dstport 4789
root@aws2:/home/ubuntu# ip a add 192.168.255.101/24 dev vxlan1
root@aws2:/home/ubuntu# ip link set up vxlan1
```
Ping failed again.

Add back the local:
```bash
ip link del vxlan1
root@aws1:/home/ubuntu# ip link add vxlan1 type vxlan local 172.31.22.156 group 239.1.1.1 dev eth0 id 100 dstport 4789
root@aws1:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@aws1:/home/ubuntu# ip link set up vxlan1
```
```bash
root@aws2:/home/ubuntu# ip link del vxlan1
root@aws2:/home/ubuntu# ip link add vxlan1 type vxlan local 172.31.23.171 group 239.1.1.1 dev eth0 id 100 dstport 4789
root@aws2:/home/ubuntu# ip a add 192.168.255.101/24 dev vxlan1
```
Ping fails. At this point, we need to demo again something that works. Add back the remote on both. I deleted the group, added back remote, and ping works again.

Try removing the remote only from aws1. Leave aws2 as is.
```bash
root@aws1:/home/ubuntu# ip link del vxlan1
root@aws1:/home/ubuntu# ip link add vxlan1 type vxlan local 172.31.22.156 dev eth0 id 100 dstport 4789
root@aws1:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@aws1:/home/ubuntu# ip link set up vxlan1
```
Ping fails. I was hoping to have a single vxlan1 on the gateway, with a single vxlan on each extra machine connecting to it. I really don't want to have to configure point-to-point connections between each machine and the gateway.

Note [here](https://community.mellanox.com/s/article/howto-configure-vxlan-for-connectx-3-pro--linux-bridge-x) I see a configuration:
```bash
ip link add vxlan10 type vxlan id 10 group 224.10.10.10 ttl 10 dev eth2
```
Just for laughs, try a config with ttl and group but no local. Tried, did not work.

The group setting probably does not work, because multicasting is not really working between VPX instances. Direct unicast will still work. But I have to wonder if it would be easier to try to get unicast working with Cyclone DDS.

[This site](https://sites.google.com/site/miclinuxcorner/technology/multicast-routing) talks about adding routes for multicast. Maybe that is the issue? I delete vxlan1 from both, add routes for multicast, and try ROS2 again.

On both instances, I did a `ip route add 224.0.0.0/4 dev eth0` and tried a talker/listener pair. No luck.
