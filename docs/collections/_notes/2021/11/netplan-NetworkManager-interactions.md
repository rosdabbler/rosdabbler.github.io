---
title: "Netplan-NetworkManager Interactions"
layout: rkjnote
author_profile: true
tags: [networking]
date: 2021-11-01
---
In my previous work, I thought I had network configuration worked out, but then I discovered that netplan was inserting an second configuration for the ethernet adapater, I know not when. I had trouble with netplan working to add the ethernet adapter to the bridge after NetworkManager was restarted.

Here I intend to investigate these interactions.

## Trying to fail

Here's what seemed to work earlier, but now fails with a restart of Networkmanager:
```bash
root@ubutower:/etc/netplan# cat 90-rosbridge.yaml
network:
  version: 2
  ethernets:
    bridge-me:
      match:
        name: "eno1*"

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
root@ubutower:/etc/netplan# systemctl restart NetworkManager
root@ubutower:/etc/netplan# netplan apply
root@ubutower:/etc/netplan# ip a
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
33: rosbridge: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000
    link/ether a6:df:66:70:76:8d brd ff:ff:ff:ff:ff:ff
root@ubutower:/etc/netplan# nmcli c
NAME                UUID                                  TYPE      DEVICE    
netplan-rosbridge   af271626-fd1c-33e2-bd7f-a0b1956032a4  bridge    rosbridge 
Wired connection 1  a72eaaf5-5b24-3911-98f0-e0d5377de748  ethernet  eno1      
netplan-bridge-me   9a4c06a7-1213-37c8-891d-25aae86703bd  ethernet  --  
root@ubutower:/run/NetworkManager/system-connections# ls
netplan-bridge-me.nmconnection  netplan-rosbridge.nmconnection
```
That is, it did not work. I changed the netplan match to:
```
      match:
        name: "eno1"
```
it started working. I changed it back to:
```
      match:
        name: "eno1*"
```
and it kept working. Making no sense. Now I can't make it fail again.

Another strange this is that I am getting a correct default route with the bridge:
```bash
root@ubutower:/etc/netplan# ip route
default via 192.168.0.11 dev rosbridge proto dhcp metric 425 
169.254.0.0/16 dev eno1 scope link metric 1000 
192.168.0.0/24 dev rosbridge proto kernel scope link src 192.168.0.55 metric 425
```
Reboot, still working, with:
```bash
root@ubutower:/etc/netplan# cat 90-rosbridge.yaml
network:
  version: 2
  ethernets:
    bridge-me:
      match:
        name: "*"

  bridges:
    rosbridge:
      mtu: 150
      interfaces: [bridge-me]
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      parameters:
        stp: false
      dhcp4: true
      dhcp6: false
```
I deleted the 'Wired connection ...' file from /etc/NetworkManager/system-connections, restarted Networkmanager, still works. Rebooted. Still works.

I wondered where the gateway address was coming from. I connected Wireshark to the ethernet adapter en01. **netplan apply** then gives a dhcp packet:
```
Next server IP address: 192.168.0.11
```
so **rosbridge** is getting the gateway from DHCP.

I tried the GUI for NetworkManager (Network Settings). It shows the ethernet search 'bridge-me' as a "Wired Connection". I can change it IP address to manual and **Apply** which causes a file to appear under **/etc/NetworkManager/system-connections** but the applied address does not stick. But this is a little dangerous as it could conflict with netplan. Who wins? I added an mtu to bridge-me in netplan, after **netplan apply** I can see it in **ip a**. Reboot.


