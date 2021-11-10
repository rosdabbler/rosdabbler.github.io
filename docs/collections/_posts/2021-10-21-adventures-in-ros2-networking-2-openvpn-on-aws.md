---
layout: single
title:  "Adventures in ROS2 Networking 2 - OpenVPN gateway"
tags: AWS networking OpenVPN
excerpt: "Here we'll setup an OpenVPN gateway server as a way to allow ROS2 nodes to run both on cloud virtual machines and in a local network with a robot."
permalink: adventures-in-ros2-networking-2
---
## Introduction

In part 1, we tried some basic networking combinations with ROS2, multiple machines, and docker.
In this part, we are going to do some initial setup with a remote cloud computer. I'll be using
Amazon Web Service (AWS) but nothing is specific to that provider, so feel free to use another
virtual server provider. The end goal of this post is an OpenVPN connection between a desktop computer and a cloud computer, pinging each other using private IP addresses in a single network. (We'll deal wih ROS2 in a subsequent post). Here's our goal for this post:

![Desktop to Cloud OpenVPN Gateway](/assets/images/OpenVpnGateways2.png "Desktop to Cloud OpenVPN Gateway")

In part 1, we showed that running nodes on two different machines in the same local network generally just works. But this fails when a ROS2 node runs on a remote machine with an internet link. Two issues cause this. First, ROS2 requires complete connectivity between two nodes over a wide range of UDP ports. Second, ROS2 node discovery works using multicast, which mostly works correctly when both nodes are on the same network segment. We cannot solve the problem with simple port forwarding at the local router or rely on NAT to maintain connectivity. We instead use an OpenVPN gateway to connect the two networks.

This is a preliminary configuration meant to introduce OpenVPN. The configuration in this post in particular will not work to provide communication between these two computers plus another on either the local network or the cloud virtual network. Those configurations will be in future posts.

## Cloud Gateway Setup

To install and test OpenVPN and ROS2 on a virtual machine, we first need a virtual machine. You'll need a test virtual machine with:
- Public IP address
- Firewall setup to accept:
    - SSH (TCP port 22)
    - OpenVPN (UDP port 1194)
    - Ping (ICMP Echo Request)
- 0.5 GiB memory, 1 CPU, 8 GB drive is sufficient
- User with sudo privileges

I have detailed instructions for starting an Amazon Web Service (AWS) EC2 instance for this demo [here](/notes/2021/10/aws-for-openvpn/ "Setting up an Amazon Web Services (AWS) virtual machine usable by OpenVPS"). If you don't use that setup, you'll need to make minor adaptations in the instructions that follows.

### Software Installation (OpenVPN, ROS2)

Ultimately I hope to be able to use a standard Docker OpenVPN container for the gateway, but as an initial test I don't want to deal with the additional complexities that Docker introduces. As a compromise, I've adapted the OpenVPN installation scripts from a [popular Docker container](https://github.com/kylemanna/docker-openvpn "OpenVPN server in a Docker container complete with an EasyRSA PKI CA"). We'll be able to use the management scripts documented there, but without Docker.

Instructions below assume that you are logged into a new virtual machine using ssh. That machine should have a minimal Ubuntu 20.04 install. The working directory should not matter (I just used ~/) but should not change during this demo.

Get and run the install script:
```bash
wget https://raw.githubusercontent.com/rosdabbler/rosdabbler.github.io/main/scripts/ain2-install.sh
source ain2-install.sh
```

### OpenVPN Configuration and Initialization
Now run the openvpn setup. We are following the instructions from [https://github.com/kylemanna/docker-openvpn](https://github.com/kylemanna/docker-openvpn), but running this locally rather than from Docker, and with small modifications to support our requirements, specifically:
* -u udp://$OPENVPN_URL: This is the URL that the client willl use to access the OpenVPN server.
* -d: Do not push a default route to clients, that is we do not want all client traffic through the VPN
* -D: Do not push new DNS server settings to clients
* -t: Use bridging rather than routing for the OpenVPN tunnel so multicast will work

You can also look at <https://github.com/kylemanna/docker-openvpn/blob/master/bin/ovpn_genconfig> to see the meaning of various setup options.

Generate the openvpn configuration (During setup, the public IP address of the Cloud Gateway was detected, and put in the variable OPENVPN_URL).
```bash
sudo ovpn_genconfig -u udp://$OPENVPN_URL -t -d -D
```

Initialize the local key provider (you'll need to enter stuff here, like creating passwords. I just used the same password in all places, and accepted the suggest default name **Easy-RSA CA** for the certificate authority.)
```bash
sudo ovpn_initpki
```

Start the OpenVPN server process (do this is a separate terminal also logged into the remote system)
```bash
sudo ovpn_run
```

Here is an example of a successful startup:
```bash
ubuntu@openvpn:~$ sudo ovpn_run
Checking IPv6 Forwarding
Sysctl error for default forwarding, please run docker with '--sysctl net.ipv6.conf.default.forwarding=1'
Sysctl error for all forwarding, please run docker with '--sysctl net.ipv6.conf.all.forwarding=1'
Running 'openvpn --config /etc/openvpn/openvpn.conf --client-config-dir /etc/openvpn/ccd --crl-verify /etc/openvpn/crl.pem '
Tue Nov  9 20:08:54 2021 OpenVPN 2.4.7 x86_64-pc-linux-gnu [SSL (OpenSSL)] [LZO] [LZ4] [EPOLL] [PKCS11] [MH/PKTINFO] [AEAD] built on Jul 19 2021
Tue Nov  9 20:08:54 2021 library versions: OpenSSL 1.1.1f  31 Mar 2020, LZO 2.10
Tue Nov  9 20:08:54 2021 Diffie-Hellman initialized with 2048 bit key
Tue Nov  9 20:08:54 2021 Outgoing Control Channel Authentication: Using 160 bit message hash 'SHA1' for HMAC authentication
Tue Nov  9 20:08:54 2021 Incoming Control Channel Authentication: Using 160 bit message hash 'SHA1' for HMAC authentication
Tue Nov  9 20:08:54 2021 TUN/TAP device tap0 opened
Tue Nov  9 20:08:54 2021 TUN/TAP TX queue length set to 100
Tue Nov  9 20:08:54 2021 /sbin/ip link set dev tap0 up mtu 1500
Tue Nov  9 20:08:54 2021 /sbin/ip addr add dev tap0 192.168.255.1/24 broadcast 192.168.255.255
Tue Nov  9 20:08:54 2021 Could not determine IPv4/IPv6 protocol. Using AF_INET
Tue Nov  9 20:08:54 2021 Socket Buffers: R=[212992->212992] S=[212992->212992]
Tue Nov  9 20:08:54 2021 UDPv4 link local (bound): [AF_INET][undef]:1194
Tue Nov  9 20:08:54 2021 UDPv4 link remote: [AF_UNSPEC]
Tue Nov  9 20:08:54 2021 GID set to nogroup
Tue Nov  9 20:08:54 2021 UID set to nobody
Tue Nov  9 20:08:54 2021 MULTI: multi_init called, r=256 v=256
Tue Nov  9 20:08:54 2021 IFCONFIG POOL: base=192.168.255.2 size=253, ipv6=0
Tue Nov  9 20:08:54 2021 Initialization Sequence Completed
```

Back on the original terminal of the Cloud Gateway, generate a client certificate without a passphrase. This is typically done for each user, but we will only be creating a single user, which we will call 'ros_desktop'. You'll be prompted to enter the password that you created earlier in ovpn_initpki.
```bash
sudo easyrsa build-client-full ros_desktop nopass
```

Retrieve the client configuration with embedded certificates into a file on the Cloud Gateway
```bash
sudo ovpn_getclient ros_desktop > ros_desktop.ovpn
```

## Desktop Gateway Setup File and Connection

Copy this .ovpn file from the remote virtual machine to some folder on your desktop. How to do this depends on your SSH setup. I just setup an SSH config file that defined a host **ros-openvpn**. If you setup your remote machine and local configuration like I did, the command to do that copy, on your desktop computer (that is, not on the Cloud Gateway), would be:
```bash
# Run me from your desktop, not from the cloud virtual machine
scp ros-openvpn:~/ros_desktop.ovpn ./
```

Still on your desktop, you'll need to install openvpn locally. That is,
```bash
sudo apt-get install openvpn
```
Now from the directory where the ros_desktop.ovpn file is located, in a new terminal, run:
```bash
sudo openvpn ros_desktop.ovpn
```

A successful connection to openvpn looks like this:
```bash
kkent@ubutower:~/openvpn$ sudo openvpn ros_desktop.ovpn
[sudo] password for kent: 
Tue Nov  9 12:12:44 2021 OpenVPN 2.4.7 x86_64-pc-linux-gnu [SSL (OpenSSL)] [LZO] [LZ4] [EPOLL] [PKCS11] [MH/PKTINFO] [AEAD] built on Jul 19 2021
Tue Nov  9 12:12:44 2021 library versions: OpenSSL 1.1.1f  31 Mar 2020, LZO 2.10
Tue Nov  9 12:12:44 2021 TCP/UDP: Preserving recently used remote address: [AF_INET]44.232.53.80:1194
Tue Nov  9 12:12:44 2021 UDP link local: (not bound)
Tue Nov  9 12:12:44 2021 UDP link remote: [AF_INET]44.232.53.80:1194
Tue Nov  9 12:12:44 2021 WARNING: 'link-mtu' is used inconsistently, local='link-mtu 1573', remote='link-mtu 1574'
Tue Nov  9 12:12:44 2021 WARNING: 'comp-lzo' is present in remote config but missing in local config, remote='comp-lzo'
Tue Nov  9 12:12:44 2021 [44.232.53.80] Peer Connection Initiated with [AF_INET]44.232.53.80:1194
Tue Nov  9 12:12:45 2021 TUN/TAP device tap0 opened
Tue Nov  9 12:12:45 2021 /sbin/ip link set dev tap0 up mtu 1500
Tue Nov  9 12:12:45 2021 /sbin/ip addr add dev tap0 192.168.255.2/24 broadcast 192.168.255.255
Tue Nov  9 12:12:45 2021 WARNING: this configuration may cache passwords in memory -- use the auth-nocache option to prevent this
Tue Nov  9 12:12:45 2021 Initialization Sequence Completed
```

## Test Connection
On the Cloud Gateway, if you run ```ip a``` you should see a tap0 adapter with an IP address 192.168.255.1
```bash
ubuntu@ip-172-31-11-150:~$ ip a show tap0
3: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 100
    link/ether 16:26:d3:e1:3f:0c brd ff:ff:ff:ff:ff:ff
    inet 192.168.255.1/24 brd 192.168.255.255 scope global tap0
       valid_lft forever preferred_lft forever
    inet6 fe80::1426:d3ff:fee1:3f0c/64 scope link 
       valid_lft forever preferred_lft forever
```

On the Desktop Gateway, ```ip a``` should also show a tap0 adapter, but with the a different IP address 192.168.255.2
```bash
kent@ubutower:~$ ip a show dev tap0
5: tap0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UNKNOWN group default qlen 100
    link/ether 06:74:23:d3:48:22 brd ff:ff:ff:ff:ff:ff
    inet 192.168.255.2/24 brd 192.168.255.255 scope global tap0
       valid_lft forever preferred_lft forever
    inet6 fe80::474:23ff:fed3:4822/64 scope link 
       valid_lft forever preferred_lft forever
```

You should be able to successfully ping the Desktop ip address from the Cloud Gateway, and vice versa.

On the Cloud Gateway:
```bash
ubuntu@ip-172-31-11-150:~$ ping 192.168.255.2
PING 192.168.255.2 (192.168.255.2) 56(84) bytes of data.
64 bytes from 192.168.255.2: icmp_seq=1 ttl=64 time=23.0 ms
64 bytes from 192.168.255.2: icmp_seq=2 ttl=64 time=11.9 ms
```

On the Desktop:
```bash
kent@ubutower:~$ ping 192.168.255.1
PING 192.168.255.1 (192.168.255.1) 56(84) bytes of data.
64 bytes from 192.168.255.1: icmp_seq=1 ttl=64 time=12.0 ms
64 bytes from 192.168.255.1: icmp_seq=2 ttl=64 time=12.0 ms
```

### Use systemctl to startup the OpenVPN server
The scripts that we used created top configure OpenVPN on the Cloud Gateway created the configuration file at **/etc/openvpn/openvpn.conf** We can use the default openvpn systemctl scripts to start openvpn using that file. On the Cloud Gateway, quit the terminal that did ```sudo ovpn_run```. Run instead to start the OpenVPN server:

```bash
sudo systemctl start openvpn@openvpn
```

You can view the logs of that startup using:
```bash
journalctl -f --unit openvpn@openvpn
```
It should look similar to before. On the Desktop Gateway, OpenVPN should be smart enough to reconnect to the Cloud Gateway assuming you are still running ```sudo openvpn ros_desktop.ovpn``` If not, quit the terminal that connected to OpenVPN, and reconnect. Pings to the opposite computers should still work.

## References

0. [Setting up an Amazon Web Services (AWS) virtual machine usable by OpenVPS](/notes/2021/10/aws-for-openvpn/)