---
title: "Persisting VXLAN configurations"
layout: archive
author_profile: true
tags: [networking]
date: 2021-11-10
---
Yesterday [I investigated](/notes/2021/11/ROS2-on-AWS/) running ROS2 nodes on AWS, and the only solution I came up with was point-to-point vxlan connections between the instances and a Cloud Gateway (though I never actually tested in on the Gateway). But now I just want to see how to persist a vxlan link. I hope I can use netplan. But [this bug report](https://bugs.launchpad.net/netplan/+bug/1764716) seems to indicate it is not supported.

OK, what else? In my EWS instances, there is an interesting file /etc/cloud.cfg  It has this:
```yaml
system_info:
  network:
    renderers: ['netplan', 'eni', 'sysconfig']
```
Other info in that file makes sense, so I will assume this does too. What is a **sysconfig** or **eni** network renderer? Found some [documentation](https://cloudinit.readthedocs.io/en/latest/topics/network-config.html) on that cloud.cfg:

> The following renderers are supported in cloud-init:
>
>    ENI
>
> /etc/network/interfaces or ENI is supported by the ifupdown package found in Alpine Linux, Debian and Ubuntu.
>
>    Sysconfig
>
> Sysconfig format is used by RHEL, CentOS, Fedora and other derivatives.

Trying to understand more how network config is done, I searched for the ethernet address in text files. **/etc** revealed little, but **/run** was useful:
```bash
root@openvpn:/etc# grep -r /run -e '0a:81:b9:3c'
/run/NetworkManager/conf.d/netplan.conf:unmanaged-devices+=mac:0a:81:b9:3c:fa:67,interface-name:eth0,
/run/udev/rules.d/99-netplan-eth0.rules:SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="0a:81:b9:3c:fa:67", NAME="eth0"
/run/cloud-init/instance-data.json:   "mac": "0a:81:b9:3c:fa:67",
/run/cloud-init/instance-data.json:      "0a:81:b9:3c:fa:67": {
/run/cloud-init/instance-data.json:       "mac": "0a:81:b9:3c:fa:67",
/run/cloud-init/instance-data-sensitive.json:   "mac": "0a:81:b9:3c:fa:67",
/run/cloud-init/instance-data-sensitive.json:      "0a:81:b9:3c:fa:67": {
/run/cloud-init/instance-data-sensitive.json:       "mac": "0a:81:b9:3c:fa:67",
/run/systemd/network/10-netplan-eth0.network:MACAddress=0a:81:b9:3c:fa:67
/run/systemd/network/10-netplan-eth0.link:MACAddress=0a:81:b9:3c:fa:67
```
So it sure looks like this cloud-init package is doing the work here. Is that how to persist vxlan? Unfortunately [this documentation](https://cloudinit.readthedocs.io/en/latest/topics/network-config-format-v1.html) lists supported config types, and VXLAN is not in the list - but VLAN is. I wonder if I can use VLAN instead of VXLAN? Earlier reading made me believe that multicast would fail, but I should test that.

But looking at [some documentation](https://developers.redhat.com/blog/2018/10/22/introduction-to-linux-interfaces-for-virtual-networking#vlan) it looks like it is a layer 2 protocol, and so far AWS has blocked all of that. I don't think this is a likely path. A better path is probably going to be to install ifupdown and see if that works. Previosuly, I tested before installing: "Tried adding a script in /etc/network/if-pre-up.d, didn't work. Googling then testing shows ifupdown not installed by default." So let be try that script.

So I did `sudo apt install ifupdown`. Here's a test script:
```bash
ubuntu@aws1:/etc/network/if-up.d$ cat rkj
#!/bin/bash
LOG=/var/log/rkj.log

echo " " >> $LOG
date >> $LOG
env >> $LOG
```
I'll reboot and see if that fires. Yes it did - but for two interfaces 'lo' and '--all' and not directly for 'eth0'. I'll also add that script to /etc/networkd-dispatcher/routable.d but with a different file name and test. Yes that worked, seemed to fire on eth0. Here is partial results:
```bash
IFACE=eth0
IP_ADDRS=172.31.22.156
ADDR=172.31.22.156
```
I assume that does not need **ifupdown** but to confirm I will test on a system without that. Yep, did not need ifupdown. So that is probably the best way to do this.

Created this script:
```bash
ubuntu@aws1:/etc/networkd-dispatcher/routable.d$ cat rovpn-vxlan 
#!/bin/bash
LOG=/var/log/rovpn-vxlan.log
REMOTE=172.31.23.171
ROVPN_IP=192.168.255.100/24
WANT_IFACE=eth0

if [[ $IFACE == $WANT_IFACE ]]; then
  echo "Configuring VXLAN to interface $IFACE" >> $LOG
  ip link add vxlan1 type vxlan remote $REMOTE local $ADDR dev $IFACE id 100 dstport 4789
  ip a add $ROVPN_IP dev vxlan1
  ip link set up vxlan1
fi
```
`reboot`

Yep that worked. Now a similar script on the second system, reboot, and test ping:
```bash
ubuntu@aws2:/etc/networkd-dispatcher/routable.d$ cat rovpn-vxlan 
#!/bin/bash
LOG=/var/log/rovpn-vxlan.log
REMOTE=172.31.22.156
ROVPN_IP=192.168.255.101/24
WANT_IFACE=eth0

if [[ $IFACE == $WANT_IFACE ]]; then
  echo "Configuring VXLAN to interface $IFACE" >> $LOG
  ip link add vxlan1 type vxlan remote $REMOTE local $ADDR dev $IFACE id 100 dstport 4789
  ip a add $ROVPN_IP dev vxlan1
  ip link set up vxlan1
fi
ubuntu@aws2:/etc/networkd-dispatcher/routable.d$ ping 192.168.255.100
PING 192.168.255.100 (192.168.255.100) 56(84) bytes of data.
64 bytes from 192.168.255.100: icmp_seq=1 ttl=64 time=0.415 ms
64 bytes from 192.168.255.100: icmp_seq=2 ttl=64 time=0.477 ms
```

OK now time to add all of this to an OpenVPN gateway system. I added a more open security group to my gateway, like I was using for test systems. At this point I also need to start using static IP addresses. I got a fresh Cloud Gateway up using the instructions in [Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway ](/adventures-in-ros2-networking-4). Now I am going to modify the bridge config in netplan.
```bash
ubuntu@rosovpn:/etc/netplan$ cat 90-rosbridge.yaml 
network:
  version: 2

  bridges:
    rosbridge:
      mtu: 1500
      parameters:
        stp: false
      addresses: [192.168.0.200/24]
      dhcp4: false
      dhcp6: false
ubuntu@rosovpn:/etc/netplan$ sudo netplan apply
ubuntu@rosovpn:/etc/netplan$ ip a show dev rosbridge
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether da:ee:bd:68:c4:69 brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.200/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::4450:dfff:fe21:f88d/64 scope link 
       valid_lft forever preferred_lft forever
```
OK having problems getting vxlan1 to work on this bridge. I'm going to disable Docker, and try to configure it locally like before. Did that, still does not work. There is some strange interaction here. I'm going to do a step-by-step install of **Adventures in ROS2 Networking 4 - Complete OpenVPN Cloud Gateway** to see if I can either get it to work, or figure out the failure point.

---
Hmmm, I accidentally applied some garbage to a new install, and wow eth0 is under rosbridge! I did not think that was possible. what the junk did is create /etc/netplan/90-rosbridge.yaml:
```yaml
network:
  version: 2
  ethernets:
    estar:
      match:
        name: "e*"

  bridges:
    rosbridge:
      mtu: 1500
      interfaces: [estar]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
then `netplan apply`. ssh crashed, but I could reconnect and then I see:
```bash
ubuntu@ip-172-31-23-155:/etc/netplan$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UP group default qlen 1000
    link/ether 02:58:77:69:d3:7d brd ff:ff:ff:ff:ff:ff
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 02:58:77:69:d3:7d brd ff:ff:ff:ff:ff:ff
    inet 172.31.23.155/20 brd 172.31.31.255 scope global dynamic rosbridge
       valid_lft 3238sec preferred_lft 3238sec
    inet6 fe80::58:77ff:fe69:d37d/64 scope link 
       valid_lft forever preferred_lft forever
```
Not sure how this helps me - but it is interesting!

Anyway, fresh install, downloaded **wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain4-install.sh** then modified to disable install of ROS2 and Docker. Ran that install script.

Now I'm going to try to manually get a vxlan link working.
```bash
root@rosovpn:/home/ubuntu# ip link add vxlan1 type vxlan remote 172.31.23.171 local 172.31.23.155 dev eth0 id 100 dstport 4789
root@rosovpn:/home/ubuntu# ip a add 192.168.255.100/24 dev vxlan1
root@rosovpn:/home/ubuntu# ip link set up vxlan1
root@rosovpn:/home/ubuntu# 
```
On the second system:
```bash
root@aws2:/home/ubuntu# ip link add vxlan1 type vxlan remote 172.31.23.155 local 172.31.23.171 dev eth0 id 100 dstport 4789
root@aws2:/home/ubuntu# ip a add 192.168.255.101/24 dev vxlan1
root@aws2:/home/ubuntu# ip link set up vxlan1
root@aws2:/home/ubuntu# ping 192.168.255.100
PING 192.168.255.100 (192.168.255.100) 56(84) bytes of data.
64 bytes from 192.168.255.100: icmp_seq=1 ttl=64 time=0.431 ms
```
OK great! Now I will continue with the OpenVPN install.
```bash
sudo ovpn_initpki
sudo easyrsa build-client-full ros_local_gateway nopass
sudo ovpn_getclient ros_local_gateway > ros_local_gateway.ovpn
sudo systemctl enable openvpn@openvpn
sudo systemctl start openvpn@openvpn
```
I can still ping across the vxlan. Good.

Now I am going to move the vxlan IP address into the middle of the local subnet 192.168.0.0/24. Let's do 192.168.0.128/26, and use .130 and .131
```bash
ubuntu@aws2:~$ sudo ip a del 192.168.255.101/24 dev vxlan1
ubuntu@aws2:~$ sudo ip a add 192.168.0.131/26 dev vxlan1
ubuntu@aws2:~$ sudo ip link set vxlan1 down
ubuntu@aws2:~$ sudo ip link set vxlan1 up
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
       valid_lft 3303sec preferred_lft 3303sec
    inet6 fe80::d:62ff:fefb:dedf/64 scope link 
       valid_lft forever preferred_lft forever
4: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue state UNKNOWN group default qlen 1000
    link/ether ba:57:3b:20:2e:1f brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.131/26 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::b857:3bff:fe20:2e1f/64 scope link 
       valid_lft forever preferred_lft forever
```
and on the gateway:
```bash
ubuntu@rosovpn:~$ sudo ip a del 192.168.255.100/24 dev vxlan1
ubuntu@rosovpn:~$ sudo ip a add 192.168.0.130/26 dev vxlan1
ubuntu@rosovpn:~$ sudo ip link set vxlan1 down
ubuntu@rosovpn:~$ sudo ip link set vxlan1 up
ubuntu@rosovpn:~$ ip a show vxlan1
4: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue state UNKNOWN group default qlen 1000
    link/ether a6:50:57:a4:d7:ea brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/26 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::a450:57ff:fea4:d7ea/64 scope link 
       valid_lft forever preferred_lft forever
```
and ping works:
```bash
ubuntu@rosovpn:~$ ping 192.168.0.131
PING 192.168.0.131 (192.168.0.131) 56(84) bytes of data.
64 bytes from 192.168.0.131: icmp_seq=1 ttl=64 time=1.79 ms
64 bytes from 192.168.0.131: icmp_seq=2 ttl=64 time=0.506 ms
```
Can the non-gateway have a broader ip subnet?
```bash
ubuntu@aws2:~$ sudo ip a del 192.168.0.131/26 dev vxlan1
ubuntu@aws2:~$ sudo ip a add 192.168.0.131/24 dev vxlan1
ubuntu@aws2:~$ sudo ip link set vxlan1 down
ubuntu@aws2:~$ sudo ip link set vxlan1 up
ubuntu@aws2:~$ ping 192.168.0.130
PING 192.168.0.130 (192.168.0.130) 56(84) bytes of data.
64 bytes from 192.168.0.130: icmp_seq=1 ttl=64 time=0.451 ms
64 bytes from 192.168.0.130: icmp_seq=2 ttl=64 time=0.529 ms
```
Yes it can. Will it still work if I move vxlan1 under the bridge?
```bash
ubuntu@rosovpn:~$ sudo ip link set vxlan1 master rosbridge
```
Yes:
```bash
ubuntu@aws2:~$ ping 192.168.0.130
PING 192.168.0.130 (192.168.0.130) 56(84) bytes of data.
64 bytes from 192.168.0.130: icmp_seq=1 ttl=64 time=0.555 ms
64 bytes from 192.168.0.130: icmp_seq=2 ttl=64 time=0.587 ms
```
Can we ping rosbridge? No.

What happens if we redefine vxlan1 to use the rosbridge IP address? I'll need to start over. Since I'm going to do that, I will use a static IP address for the bridge.
```yaml
ubuntu@rosovpn:/etc/netplan$ cat 90-rosbridge.yaml 
network:
  version: 2

  bridges:
    rosbridge:
      mtu: 1500
      parameters:
        stp: false
      addresses: [192.168.0.201/24]
      dhcp4: false
      dhcp6: false
```
and that works, from desktop I can ping 192.168.0.201. The desktop can also ping vxlan1 (192.168.0.130)! But not 192.168.0.131 (vxlan1 is not currently under the bridge). If I set vxlan1 under the bridge, then I CAN ping the second AWS system from the desktop! And the second system can ping the desktop!
I have:
```bash
ubuntu@rosovpn:/etc/netplan$ sudo ip link set vxlan1 master rosbridge
ubuntu@rosovpn:/etc/netplan$ ip a
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
    inet6 ::1/128 scope host 
       valid_lft forever preferred_lft forever
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 9001 qdisc fq_codel state UP group default qlen 1000
    link/ether 02:58:77:69:d3:7d brd ff:ff:ff:ff:ff:ff
    inet 172.31.23.155/20 brd 172.31.31.255 scope global dynamic eth0
       valid_lft 3278sec preferred_lft 3278sec
    inet6 fe80::58:77ff:fe69:d37d/64 scope link 
       valid_lft forever preferred_lft forever
3: rosbridge: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 02:08:0c:31:aa:0c brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.201/24 brd 192.168.0.255 scope global rosbridge
       valid_lft forever preferred_lft forever
    inet6 fe80::e4b7:32ff:fe5d:c14/64 scope link 
       valid_lft forever preferred_lft forever
4: vxlan1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 8951 qdisc noqueue master rosbridge state UNKNOWN group default qlen 1000
    link/ether a6:50:57:a4:d7:ea brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.130/26 scope global vxlan1
       valid_lft forever preferred_lft forever
    inet6 fe80::a450:57ff:fea4:d7ea/64 scope link 
       valid_lft forever preferred_lft forever
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel master rosbridge state UNKNOWN group default qlen 100
    link/ether 02:08:0c:31:aa:0c brd ff:ff:ff:ff:ff:ff
    inet6 fe80::8:cff:fe31:aa0c/64 scope link 
       valid_lft forever preferred_lft forever
Kernel IP routing table
Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
0.0.0.0         172.31.16.1     0.0.0.0         UG    100    0        0 eth0
172.31.16.0     0.0.0.0         255.255.240.0   U     0      0        0 eth0
172.31.16.1     0.0.0.0         255.255.255.255 UH    100    0        0 eth0
192.168.0.0     0.0.0.0         255.255.255.0   U     0      0        0 rosbridge
192.168.0.128   0.0.0.0         255.255.255.192 U     0      0        0 vxlan1
```
How about ROS2? Setup a listener on the desktop, and a talker on AWS2. And it works!

Time to get serious about IP addresses. My old IP address assignment page is REALLY out of date. I'll do a new one somewhere private. (Note to self: See ~/Documents/IP-Address-Plan.md)