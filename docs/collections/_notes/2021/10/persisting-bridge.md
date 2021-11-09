---
title: "Persisting a Docker bridge"
layout: archive
author_profile: true
tags: [docker, networking]
date: 2021-10-26
---
Since the robot will be running from a Docker-compatible bridge, ideally that bridge would exist even without Docker running. Let me test how to do that.

## How to manage persistent connections
First question, what mechanism is actually being used on my computers to manage the network? I'll check two computers: my desktop running Ubuntu 20.04, and an AWS remote running Ubuntu 20.04 minimal.

What does systemd see? Desktop:
```bash
kent@ubutower:/etc/NetworkManager$ systemctl list-unit-files | grep etwork
network-manager.service                    enabled         enabled      
networkd-dispatcher.service                enabled         enabled      
NetworkManager-dispatcher.service          enabled         enabled      
NetworkManager-wait-online.service         enabled         enabled      
NetworkManager.service                     enabled         enabled      
systemd-network-generator.service          disabled        enabled      
systemd-networkd-wait-online.service       enabled-runtime enabled      
systemd-networkd.service                   enabled-runtime enabled      
systemd-networkd.socket                    disabled        enabled      
network-online.target                      static          enabled      
network-pre.target                         static          disabled     
network.target                             static          disabled     
kent@ubutower:/etc/NetworkManager$ 
```
AWS:
```bash
ubuntu@openvpn:/etc/netplan$ systemctl list-unit-files | grep etwork
network-manager.service                        enabled         enabled      
networkd-dispatcher.service                    enabled         enabled      
NetworkManager-dispatcher.service              enabled         enabled      
NetworkManager-wait-online.service             enabled         enabled      
NetworkManager.service                         enabled         enabled      
systemd-network-generator.service              disabled        enabled      
systemd-networkd-wait-online.service           enabled         enabled      
systemd-networkd.service                       enabled         enabled      
systemd-networkd.socket                        enabled         enabled      
network-online.target                          static          enabled      
network-pre.target                             static          disabled     
network.target                                 static          disabled
```
That did not help, virtually the same. I'm pretty sure that both use netplan. Checking its config.

Desktop:
```bash
kent@ubutower:/etc/netplan$ ls
01-network-manager-all.yaml
```
AWS:
```bash
ubuntu@openvpn:/etc/netplan$ ls
50-cloud-init.yaml
```
So I am guessing that while the AWS system has NetworkManager installed, it is not actually using it. Let me try to confirm that by trying a NetManager config. First, show NetworkManager at work like this:
```bash
kent@ubutower:/~$ sudo nmcli con add type bridge ifname brtest
Connection 'bridge-brtest' (6680a86e-3138-4421-955c-92212d5671aa) successfully added.
```
**ip a** shows the bridge. Reboot. After reboot, the bridge is recreated:
```bash
kent@ubutower:~$ ip a show brtest
3: brtest: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000
    link/ether 72:64:de:24:af:9c brd ff:ff:ff:ff:ff:ff
```
Try the same thing on my AWS Ubuntu instance.
```bash
ubuntu@openvpn:~$ sudo nmcli con add type bridge ifname brtest
Connection 'bridge-brtest' (7f364d9c-1ee6-4bc1-a32d-00919e959d52) successfully added.
```
This time, unlike the desktop, **ip a** does not show the connection. I can see that NetworkManager knows about it:
```bash
ubuntu@openvpn:~$ sudo nmcli c
NAME           UUID                                  TYPE    DEVICE 
bridge-brtest  7f364d9c-1ee6-4bc1-a32d-00919e959d52  bridge  --  
```
but trying to bring it up gives me:
```bash
ubuntu@openvpn:~$ sudo nmcli c up bridge-brtest
Error: Connection activation failed: Activation failed because the device is unmanaged
```
So the conclusion I reach is this: **Using a netplan hook is the way to persist a connection on both my Desktop as well as on an AWS clould instance**

Use **sudo nmcli c delete bridge-brtest** to delete the bridge.

## Netplan bridge survival with Docker

The general plan for OpenVPN networking with ROS2 is to use a Docker bridge as the --net type, then add the OpenVPN tap0 link to that bridge. I need to see how Docker interacts with all of that. I recall reading somewhere that if Docker sees a bridge already in existence it will not recreate it. So the general plan is to persistently create the bridge, then link it to Docker.

I'm going to create a bridge using netplan that adds the same bridge that Docker would create, so that it works with Docker unloaded. Create a file **/etc/netplan/90-rosbridge.yaml** with this content, which I adapted from [here](https://www.techrepublic.com/article/how-to-create-a-bridge-network-on-linux-with-netplan/):
```
network:
  version: 2
  renderer: networkd
  bridges:
    rosbridge:
      addresses: [10.231.168.1/20]
      mtu: 1500
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: false
      dhcp6: false
```

Use **sudo netplan apply** to get netplan to act on the change. That created the bridge:
```bash
kent@ubutower:/etc/netplan$ ip a show rosbridge
4: rosbridge: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000
    link/ether e2:47:07:91:c5:36 brd ff:ff:ff:ff:ff:ff
    inet 10.231.168.1/20 brd 10.231.175.255 scope global rosbridge
       valid_lft forever preferred_lft forever
```
Now use docker to create the same bridge.
```bash
kent@ubutower:/etc/netplan$ docker network create --driver=bridge \
> --subnet=10.231.160.0/20 \
> --gateway=10.231.168.1 \
> --ip-range=10.231.168.128/29 \
> --opt com.docker.network.bridge.name=rosbridge \
> rosbridge
```
That worked. Check that it works:
```bash
kent@ubutower:~$ docker run -it --rm --net rosbridge busybox
/ # ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
8: eth0@if9: <BROADCAST,MULTICAST,UP,LOWER_UP,M-DOWN> mtu 1500 qdisc noqueue 
    link/ether 02:42:0a:e7:a8:80 brd ff:ff:ff:ff:ff:ff
    inet 10.231.168.128/20 brd 10.231.175.255 scope global eth0
       valid_lft forever preferred_lft forever
/ # 
```
Yes. Now, I'm going to reboot with Docker disabled ... OK I'm back. **ip a** shows the rosbridge, but I have a new problem: routes are messed up.
```bash
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
10.231.160.0    0.0.0.0         255.255.240.0   U     0      0        0 rosbridge
kent@ubutower:~$ ping 8.8.8.8
ping: connect: Network is unreachable
```
That's bad. Removing the new file from /etc/netplan then **sudo netplan apply** did not fix the problem. **ip a** shows that the ethernet link is down. **sudo ip link set up eno1** does not help. Hardware issue? Let me power down ... Yep that fixed it. Weird. Add back the rosbridge create file to /etc/netplan, and reboot again.

Problem reappeared. Removed /etc/netplan rosbridge file, rebooted, this time ethernet came back without a power down. I tried a few more times, when the new /etc/netplan file is there, ethernet fails on reboot.

On a hunch, I remove the renderer line from the /etc/netplan/ file. **sudo netplan apply** still restores the bridge. How about on reboot? ... That worked! So apparently, the renderer line in my first attempt:
```
network:
  version: 2
  renderer: networkd
```
removed NetworkManager from the startup. Don't want that line!

Back to docker. Use systemctl to start it. I can do **docker run --net rosbridge ...** successfully.

Now on the OpenVPN. What I need to do is to add the tap0 device to rosbridge when OpenVPN is connected. I got my AWS openvpn gateway up, so the tap0 interface exists on desktop and remote, and I can ping the remote from the desktop:
```bash
kent@ubutower:~$ ip a show dev tap0
7: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 100
    link/ether 1a:a0:1b:22:8e:3e brd ff:ff:ff:ff:ff:ff
    inet 10.231.160.2/20 brd 10.231.175.255 scope global tap0
       valid_lft forever preferred_lft forever
    inet6 fe80::18a0:1bff:fe22:8e3e/64 scope link 
       valid_lft forever preferred_lft forever

kent@ubutower:~$ ping 10.231.161.1
PING 10.231.161.1 (10.231.161.1) 56(84) bytes of data.
64 bytes from 10.231.161.1: icmp_seq=1 ttl=64 time=11.6 ms
64 bytes from 10.231.161.1: icmp_seq=2 ttl=64 time=12.1 ms
```
Now I want to move the tap0 device under the docker bridge.
```bash
kent@ubutower:~$ sudo ip link set master rosbridge dev tap0^C
kent@ubutower:~$ ip a show dev tap0
7: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether 1a:a0:1b:22:8e:3e brd ff:ff:ff:ff:ff:ff
    inet 10.231.160.2/20 brd 10.231.175.255 scope global tap0
       valid_lft forever preferred_lft forever
    inet6 fe80::18a0:1bff:fe22:8e3e/64 scope link 
       valid_lft forever preferred_lft forever
```
Note the 'master rosbridge' under tap0. Now from the remote system, I can ping Docker bridge:
```bash
ubuntu@openvpn:~$ ping 10.231.168.1
PING 10.231.168.1 (10.231.168.1) 56(84) bytes of data.
64 bytes from 10.231.168.1: icmp_seq=1 ttl=64 time=23.1 ms
64 bytes from 10.231.168.1: icmp_seq=2 ttl=64 time=11.8 ms
```

I created a docker container on the desktop:
```bash
docker run -it --rm --net rosbridge busybox
```
and tried to ping the remote. Did not work.Checking the routing table:
```bash
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    100    0        0 eno1
10.231.160.0    0.0.0.0         255.255.240.0   U     0      0        0 tap0
10.231.160.0    0.0.0.0         255.255.240.0   U     425    0        0 rosbridge
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 rosbridge
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```
we still have the tap0 route. Remove it:
```bash
kent@ubutower:~$ sudo ip route del 10.231.160.0/20 dev tap0
kent@ubutower:~$ route -n
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         192.168.0.11    0.0.0.0         UG    100    0        0 eno1
10.231.160.0    0.0.0.0         255.255.240.0   U     425    0        0 rosbridge
169.254.0.0     0.0.0.0         255.255.0.0     U     1000   0        0 rosbridge
172.17.0.0      0.0.0.0         255.255.0.0     U     0      0        0 docker0
192.168.0.0     0.0.0.0         255.255.255.0   U     100    0        0 eno1
```
and now the ping works:
```bash
kent@ubutower:~$ docker run -it --rm --net rosbridge busybox
/ # ping 10.231.161.1
PING 10.231.161.1 (10.231.161.1): 56 data bytes
64 bytes from 10.231.161.1: seq=0 ttl=64 time=20.824 ms
64 bytes from 10.231.161.1: seq=1 ttl=64 time=10.746 ms
```
Checking the [OpenVPN manual](https://openvpn.net/community-resources/reference-manual-for-openvpn-2-4/) I tried adding these commands to the .ovpn file used to start openvpn on the desktop (though not sure of the syntax):
```
ifconfig-noexec
up 'ip route del 10.231.160.0/20 dev tap0 && ip link set tap0 up'
```
Tried that, grrr, *Options error: --up script fails with 'ip': No such file or directory (errno=2)
*. OK:
```
ifconfig-noexec
up '/usr/sbin/ip route del 10.231.160.0/20 dev tap0 && /usr/sbin/ip link set tap0 up'
```
Grrr, *Fri Oct 29 11:40:20 2021 WARNING: External program may not be called unless '--script-security 2' or higher is enabled. See --help text or man page for detailed info.*
OK again:
```
script-security 2
ifconfig-noexec
up 'ip route del 10.231.160.0/20 dev tap0 && ip link set tap0 up'
```
Grrr, *Error: either "to" is duplicate, or "&&" is a garbage.*
OK, let's let bash run it:
```
script-security 2
ifconfig-noexec
up "/bin/bash -c 'route del 10.231.160.0/20 dev tap0 && ip link set tap0 up'"
```
Still an error, wrong command. Again:
```
script-security 2
ifconfig-noexec
up "/bin/bash -c 'ip link set master rosbridge dev tap0 && ip link set tap0 up'"
```
and this works, a docker with --net rosbridge on the desktop can ping the remote tap0 interface.
