---
title: "AIN5 - Trying to get the scripts working"
layout: rkjnote
author_profile: true
tags: [networking, aws]
date: 2021-11-12github 
---
Earlier I installed a vxlan between two AWS instances along with an OpenVPN gateway. I managed to get it to work without Docker, but I went ahead anyway and did install scripts. Tried them they did not work. So I need to sort that out.

Last commit which was used for the install scripts was 32d9fde to rosdabbler.github.io

### Disable Docker, reboot.
Tried this, no help. So I am going to terminate these instances, and re-install without installing ROS2 and Docker, and make sure that works.

Note to self: AMI image I used can be found searching for 070f417

```bash
ROSVPN_SKIP_ROS2=true
ROSVPN_SKIP_DOCKER=true
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain5-gw-install.sh
source ain5-gw-install.sh
```

Second system:
```bash
ubuntu@ip-172-31-1-12:~$ ROSVPN_SKIP_ROS2=true
ubuntu@ip-172-31-1-12:~$ ROSVPN_SKIP_DOCKER=true
ubuntu@ip-172-31-1-12:~$ wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain5-vx-install.sh
ubuntu@ip-172-31-1-12:~$ source ain5-vx-install.sh 
```

These did not work. ROS2 install started, plus had later failures. Debugging. New instances, modified install scripts setting variables to not install, this time install completed cleanly.

But still the vxlan does not work.

I changed security groups to the one allow all traffic between nodes. No help. Rebooted. No help

I tried `sudo iptables -L -v` in both systems. Second system did not have iptables installed! `sudo apt install iptables` No help. Also, note all tables are empty with default ACCEPT on both systems. So what is different from the previous success?

OK I see that I had an error in the AWS secutiry group (I had an older private subnet isted). Fixed that. No help.

Installed `sudo apt install tcpdump` both systems. `sudo tcpdump -n -i any -e not port 22` shows me details I need. I can see vxlan1 working, ping request and ARP details get to gateway syste, but that system makes no attempt to reply. What makes ping work is, on the gateway, `sudo ip link set vxlan1 nomaster` that is remove it from the bridge.

One difference I see is that yesterday, rosbridge shared mac address with tap0. Today, it is sharing with vxlan1. Not sure if that matters though.

This is what the links looked like when it did not work:
```bash
ubuntu@rosvpn-gw:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:02:55:7b:5b:45 brd ff:ff:ff:ff:ff:ff
    inet 172.31.1.11/24 brd 172.31.1.255 scope global dynamic eth0
       valid_lft 1862sec preferred_lft 1862sec
    inet6 fe80::2:55ff:fe7b:5b45/64 scope link 
       valid_lft forever preferred_lft forever
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 3a:90:fe:9c:31:88 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::8416:abff:fea1:c0a/64 scope link 
       valid_lft forever preferred_lft forever
4: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether e2:e2:d6:bc:9c:4f brd ff:ff:ff:ff:ff:ff
    inet6 fe80::e0e2:d6ff:febc:9c4f/64 scope link 
       valid_lft forever preferred_lft forever
5: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue master rosbridge state UNKNOWN group default qlen 1000
    link/ether 3a:90:fe:9c:31:88 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.145/30 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::3890:feff:fe9c:3188/64 scope link 
       valid_lft forever preferred_lft forever
```

I came across [this reference](https://backreference.org/2010/07/28/linux-bridge-mac-addresses-and-dynamic-ports/) which has:

> Now, by default bridge interfaces in Linux use, for their MAC address, the lowest MAC address among the enslaved interfaces. So if the newly created interface has a lower MAC, the bridge changes its MAC address and uses that of the new interface.  ...  I found out that if the bridge's MAC address is forced to a specific value, the bridge "remembers" that and makes the address permanent. But there's a caveat: the address must belong to one of the devices enslaved to the bridge. In our example (and probably also in the most common case), the ideal candidate is obviously eth0, which is permanently enslaved.

That was 11 years ago. Let me play a little with this.

Put vxlan back under the bridge:
```bash
ubuntu@rosvpn-gw:~$ sudo ip link set vxlan1 master rosbridge
ubuntu@rosvpn-gw:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:02:55:7b:5b:45 brd ff:ff:ff:ff:ff:ff
    inet 172.31.1.11/24 brd 172.31.1.255 scope global dynamic eth0
       valid_lft 2766sec preferred_lft 2766sec
    inet6 fe80::2:55ff:fe7b:5b45/64 scope link 
       valid_lft forever preferred_lft forever
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 3a:90:fe:9c:31:88 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::8416:abff:fea1:c0a/64 scope link 
       valid_lft forever preferred_lft forever
4: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether e2:e2:d6:bc:9c:4f brd ff:ff:ff:ff:ff:ff
    inet6 fe80::e0e2:d6ff:febc:9c4f/64 scope link 
       valid_lft forever preferred_lft forever
5: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue master rosbridge state UNKNOWN group default qlen 1000
    link/ether 3a:90:fe:9c:31:88 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.145/30 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::3890:feff:fe9c:3188/64 scope link 
       valid_lft forever preferred_lft forever
```

Weird, it worked briefly then stopped:
```bash
ubuntu@rosvpn-second:~$ ping 192.168.0.145
PING 192.168.0.145 (192.168.0.145) 56(84) bytes of data.
64 bytes from 192.168.0.145: icmp_seq=1 ttl=64 time=0.629 ms
64 bytes from 192.168.0.145: icmp_seq=2 ttl=64 time=0.541 ms
^C
--- 192.168.0.145 ping statistics ---
2 packets transmitted, 2 received, 0% packet loss, time 1027ms
rtt min/avg/max/mdev = 0.541/0.585/0.629/0.044 ms
ubuntu@rosvpn-second:~$ ping 192.168.0.145
PING 192.168.0.145 (192.168.0.145) 56(84) bytes of data.
^C
--- 192.168.0.145 ping statistics ---
10 packets transmitted, 0 received, 100% packet loss, time 9195ms
```

Try setting manually mac address of bridge to tap0 value. At first. no luch. But I togglled vxlan1 out and in and after a few seconds it started to work:

```bash
sudo ip link set rosbridge address e2:e2:d6:bc:9c:4f
sudo ip link set vxlan1 nomaster
sudo ip link set vxlan1 master rosbridge
```
But then after more time, it stopped working again. Boo.

Let me try some alternate approaches. Next: GRE

gateway:
```bash
ubuntu@rosvpn-gw:~$ sudo ip link add gre1 type gre remote 172.31.1.12 local 172.31.1.11 dev eth0
ubuntu@rosvpn-gw:~$ sudo ip link set gre1 up
ubuntu@rosvpn-gw:~$ sudo ip a add 192.168.0.145/30 dev gre1
ubuntu@rosvpn-gw:~$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:02:55:7b:5b:45 brd ff:ff:ff:ff:ff:ff
    inet 172.31.1.11/24 brd 172.31.1.255 scope global dynamic eth0
       valid_lft 2662sec preferred_lft 2662sec
    inet6 fe80::2:55ff:fe7b:5b45/64 scope link 
       valid_lft forever preferred_lft forever
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether e2:e2:d6:bc:9c:4f brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::e0e2:d6ff:febc:9c4f/64 scope link 
       valid_lft forever preferred_lft forever
4: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether e2:e2:d6:bc:9c:4f brd ff:ff:ff:ff:ff:ff
    inet6 fe80::e0e2:d6ff:febc:9c4f/64 scope link 
       valid_lft forever preferred_lft forever
11: gre0@NONE: <NOARP> mtu 1476 qdisc noop state DOWN group default qlen 1000
    link/gre 0.0.0.0 brd 0.0.0.0
12: gretap0@NONE: <BROADCAST,MULTICAST> mtu 1462 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
13: erspan0@NONE: <BROADCAST,MULTICAST> mtu 1450 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
14: gre1@eth0: <POINTOPOINT,NOARP,UP,LOWER_UP> mtu 8977 qdisc noqueue state UNKNOWN group default qlen 1000
    link/gre 172.31.1.11 peer 172.31.1.12
    inet 192.168.0.145/30 scope global gre1
       valid_lft forever preferred_lft forever
    inet6 fe80::5efe:ac1f:10b/64 scope link 
       valid_lft forever preferred_lft forever
```

second:
```bash
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ sudo ip link del vxlan1
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ sudo ip link add gre1 type gre remote 172.31.1.11 local 172.31.1.12 dev eth0
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ sudo ip link set gre1 up
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ sudo ip a add 192.168.0.146/30 dev gre1
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:9f:dd:dd:74:5f brd ff:ff:ff:ff:ff:ff
    inet 172.31.1.12/24 brd 172.31.1.255 scope global dynamic eth0
       valid_lft 2464sec preferred_lft 2464sec
    inet6 fe80::9f:ddff:fedd:745f/64 scope link 
       valid_lft forever preferred_lft forever
4: gre0@NONE: <NOARP> mtu 1476 qdisc noop state DOWN group default qlen 1000
    link/gre 0.0.0.0 brd 0.0.0.0
5: gretap0@NONE: <BROADCAST,MULTICAST> mtu 1462 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
6: erspan0@NONE: <BROADCAST,MULTICAST> mtu 1450 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
7: gre1@eth0: <POINTOPOINT,NOARP,UP,LOWER_UP> mtu 8977 qdisc noqueue state UNKNOWN group default qlen 1000
    link/gre 172.31.1.12 peer 172.31.1.11
    inet 192.168.0.146/30 scope global gre1
       valid_lft forever preferred_lft forever
    inet6 fe80::5efe:ac1f:10c/64 scope link 
       valid_lft forever preferred_lft forever
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.1.1      0.0.0.0         UG    100    0        0 eth0
172.31.1.0      0.0.0.0         255.255.255.0   U     0      0        0 eth0
172.31.1.1      0.0.0.0         255.255.255.255 UH    100    0        0 eth0
192.168.0.144   0.0.0.0         255.255.255.252 U     0      0        0 gre1
ubuntu@rosvpn-second:/etc/networkd-dispatcher/routable.d$ ping 192.168.0.145
PING 192.168.0.145 (192.168.0.145) 56(84) bytes of data.
64 bytes from 192.168.0.145: icmp_seq=1 ttl=64 time=0.476 ms
64 bytes from 192.168.0.145: icmp_seq=2 ttl=64 time=0.472 ms
```

and:
```bash
ubuntu@rosvpn-gw:~$ ping 192.168.0.146
PING 192.168.0.146 (192.168.0.146) 56(84) bytes of data.
64 bytes from 192.168.0.146: icmp_seq=1 ttl=64 time=0.407 ms
64 bytes from 192.168.0.146: icmp_seq=2 ttl=64 time=0.546 ms
```
Now add to bridge:
```bash
ubuntu@rosvpn-gw:~$ sudo ip link set gre1 master rosbridge
RTNETLINK answers: Invalid argument
```
Hmmm. I think what I really need is a gretap. Retry:
```
sudo ip link del gre1
```
That worked, but there are leftover interfaces shown by `ip a`:
```bash
11: gre0@NONE: <NOARP> mtu 1476 qdisc noop state DOWN group default qlen 1000
    link/gre 0.0.0.0 brd 0.0.0.0
12: gretap0@NONE: <BROADCAST,MULTICAST> mtu 1462 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
13: erspan0@NONE: <BROADCAST,MULTICAST> mtu 1450 qdisc noop state DOWN group default qlen 1000
    link/ether 00:00:00:00:00:00 brd ff:ff:ff:ff:ff:ff
```
So I rebooted. Disable vxlan1 in the process:
```
ubuntu@rosvpn-gw:~$ cd /etc/networkd-dispatcher/routable.d/
ubuntu@rosvpn-gw:/etc/networkd-dispatcher/routable.d$ ls
vxlan1
ubuntu@rosvpn-gw:/etc/networkd-dispatcher/routable.d$ sudo rm vxlan1 
ubuntu@rosvpn-gw:/etc/networkd-dispatcher/routable.d$ sudo reboot
```
Do on both systems. After reboot:
```bash
ubuntu@rosvpn-gw:~$ sudo ip link add gre1 type gretap remote 172.31.1.12 local 172.31.1.11 dev eth0
ubuntu@rosvpn-gw:~$ sudo ip link set gre1 up
ubuntu@rosvpn-gw:~$ sudo ip a add 192.168.0.145/30 dev gre1
ubuntu@rosvpn-gw:~$ ip link show gre1
8: gre1@eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8963 qdisc fq_codel state UNKNOWN mode DEFAULT group default qlen 1000
    link/ether ee:ac:bc:d3:f3:43 brd ff:ff:ff:ff:ff:ff
```
I still get those other funny interfaces.
On second:
```bash
ubuntu@rosvpn-second:~$ sudo ip link add gre1 type gretap remote 172.31.1.11 local 172.31.1.12 dev eth0
ubuntu@rosvpn-second:~$ sudo ip link set gre1 up
ubuntu@rosvpn-second:~$ sudo ip a add 192.168.0.146/30 dev gre1
ubuntu@rosvpn-second:~$ ping 192.168.0.145
PING 192.168.0.145 (192.168.0.145) 56(84) bytes of data.
64 bytes from 192.168.0.145: icmp_seq=1 ttl=64 time=0.891 ms
64 bytes from 192.168.0.145: icmp_seq=2 ttl=64 time=0.510 ms
```
OK so far so good. Now the bridge test. On gateway:
```bash
sudo ip link set gre1 master rosbridge
```
And on remote:
```bash
ping 192.168.0.145
```
Same behavior as before. I got 8 pings to work, then no more.

OK, next step is to remove the IP addresses from the gre1. Not sure they are needed. Type that:
```bash
ubuntu@rosvpn-second:~$ sudo ip a del 192.168.0.146/30 dev gre1
ubuntu@rosvpn-second:~$ sudo ip a add 192.168.0.146/24 dev gre1
```
```bash
sudo ip a del 192.168.0.145/30 dev gre1
PING 192.168.0.146 (192.168.0.146) 56(84) bytes of data.
64 bytes from 192.168.0.146: icmp_seq=1 ttl=64 time=0.790 ms
64 bytes from 192.168.0.146: icmp_seq=2 ttl=64 time=0.535 ms
```
Worked! And it kept working! Yay! So I started ros galactic desktop install on each. Tried talker/listener (with gre1 as the device on the second) and it worked!

Probably vxlan works with the same config. I remove the gre devives, and recreated the vxlan devices.

Gateway:
```bash
sudo ip link del gre1
sudo ip link add vxlan1 type vxlan remote 172.31.1.12 local 172.31.1.11 dev eth0 id 100 dstport 4789
sudo ip link set vxlan1 up
sudo ip link set vxlan1 master rosbridge
```
Second:
```bash
sudo ip link del gre1
sudo ip link add vxlan1 type vxlan remote 172.31.1.11 local 172.31.1.12 dev eth0 id 100 dstport 4789
sudo ip link set vxlan1 up
sudo ip a add 192.168.0.146/24 dev vxlan1
```
Yes that also worked with ping. Did not test with ros2.

As to **gretap** vs **vxlan**, two advantages of **gretap**: Lower byte overhead per packet, plus supported by netplan. Let my try a netplan installation. I rebooted both systems.

Gateway: modify /etc/netplan/90-rosbridge.yaml
```yaml
network:
  version: 2

  tunnels:
    gre1:
      renderer: networkd
      mode: gretap
      remote: 172.31.1.12
      local: 172.31.1.11

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [gre1]
      parameters:
        stp: false
      addresses: [192.168.0.130/24]
      dhcp4: false
      dhcp6: false
```
Second: Create file 90-gre1.yaml
```yaml
network:
  version: 2

  tunnels:
    gre1:
      renderer: networkd
      mode: gretap
      remote: 172.31.1.11
      local: 172.31.1.12
      addresses: [192.168.0.146/24]
```
I need to try this with Docker. Installed on both systems. Yep still works. Now clean up the scripts and try a fresh install from scripts.