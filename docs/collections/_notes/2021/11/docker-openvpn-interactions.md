---
title: "Docker-OpenVPN interactions"
layout: archive
author_profile: true
tags: [openvpn, docker]
date: 2021-11-04
---
I thought I was done with my OpenVPN configuration - but then I had failures that I traced back to being caused by starting Docker on the local gateway. Stopping Docker does not help, nor removing the bridge and ```netplan apply``` to recreate.

Symptoms:
- DHCP failed for the cloud OpenVPN connection
- Ping from a second local system to the cloud bridge fails (gateway-to-gateway ping works if I manually assign an IP)

I can get things working without Docker installed, and once I ```sudo systemctl start docker``` pings stop working from the second local system.

Docker leaves a configuration in **iptables** even after I stop it. Is that the issue?

Prior to starting Docker, iptables shows, with the second computer pinging the cloud gateway:
```bash
kent@ubutower:~$ sudo iptables -L -v
Chain INPUT (policy ACCEPT 6182 packets, 28M bytes)
 pkts bytes target     prot opt in     out     source               destination         

Chain FORWARD (policy ACCEPT 0 packets, 0 bytes)
 pkts bytes target     prot opt in     out     source               destination         

Chain OUTPUT (policy ACCEPT 6001 packets, 769K bytes)
 pkts bytes target     prot opt in     out     source               destination
 ```
 After Docker is started,
 ```bash
 kent@ubutower:~$ sudo iptables -L -v
Chain INPUT (policy ACCEPT 224 packets, 26205 bytes)
 pkts bytes target     prot opt in     out     source               destination         

Chain FORWARD (policy DROP 23 packets, 1828 bytes)
 pkts bytes target     prot opt in     out     source               destination         
   23  1828 DOCKER-USER  all  --  any    any     anywhere             anywhere            
   23  1828 DOCKER-ISOLATION-STAGE-1  all  --  any    any     anywhere             anywhere            
    0     0 ACCEPT     all  --  any    docker0  anywhere             anywhere             ctstate RELATED,ESTABLISHED
    0     0 DOCKER     all  --  any    docker0  anywhere             anywhere            
    0     0 ACCEPT     all  --  docker0 !docker0  anywhere             anywhere            
    0     0 ACCEPT     all  --  docker0 docker0  anywhere             anywhere            

Chain OUTPUT (policy ACCEPT 252 packets, 25258 bytes)
 pkts bytes target     prot opt in     out     source               destination         

Chain DOCKER (1 references)
 pkts bytes target     prot opt in     out     source               destination         

Chain DOCKER-ISOLATION-STAGE-1 (1 references)
 pkts bytes target     prot opt in     out     source               destination         
    0     0 DOCKER-ISOLATION-STAGE-2  all  --  docker0 !docker0  anywhere             anywhere            
   23  1828 RETURN     all  --  any    any     anywhere             anywhere            

Chain DOCKER-ISOLATION-STAGE-2 (1 references)
 pkts bytes target     prot opt in     out     source               destination         
    0     0 DROP       all  --  any    docker0  anywhere             anywhere            
    0     0 RETURN     all  --  any    any     anywhere             anywhere            

Chain DOCKER-USER (1 references)
 pkts bytes target     prot opt in     out     source               destination         
   23  1828 RETURN     all  --  any    any     anywhere             anywhere  
```
If I flush the FORWARD chain, and change the policy to ACCEPT, ping starts working.
```bash
sudo iptables -F FORWARD
sudo iptables -P FORWARD ACCEPT
```
What is strange is that prior to Docker, the bridged connections were not going through FORWARD, but after they are. A logged item in kern.log gave me a hint that Docker enabled bridged routing or some such. Upon further investigation, this might be br_netfilter. ebtables is empty, that's not it.

Check this out after Docker:
```bash
kent@ubutower:~$ sysctl net.bridge
net.bridge.bridge-nf-call-arptables = 1
net.bridge.bridge-nf-call-ip6tables = 1
net.bridge.bridge-nf-call-iptables = 1
net.bridge.bridge-nf-filter-pppoe-tagged = 0
net.bridge.bridge-nf-filter-vlan-tagged = 0
net.bridge.bridge-nf-pass-vlan-input-dev = 0
```
Let me reboot and see what changed ... after reboot, I get:
```bash
root@ubutower:~# sysctl net.bridge
sysctl: cannot stat /proc/sys/net/bridge: No such file or directory
```
So I am onto something here. After ```modprobe br_netfilter``` I get the same listing for net.bridge as after Docker networking.

I can ```modprobe -r br_netfilter``` and the ```sysctl net.bridge``` variables disappear again. So does my bridge in ```ip a``` so I have to ```netplan apply``` after removing br_netfilter to get it back again.

I can also confirm that prior to ```modprobe br_netfilter``` the second computer ping of the cloud gateway show up in the iptables INPUT chain. After, they show up in the FORWARD chain. So that explains what I am seeing after Docker is installed.

So let's see what these dropped packets look like. Start Docker, the ping stops working. I need to add to DOCKER-USER to allow them. First log them:
```bash
iptables -I DOCKER-USER -j LOG --log-level 6
```
Looking at /var/log/kern.log, here are sample packets:
```log
Nov  4 23:38:01 ubutower kernel: [ 1526.472022] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=22:7c:e3:ac:d3:c7:40:16:7e:84:26:87:08:00 SRC=192.168.0.210 DST=192.168.0.90 LEN=84 TOS=0x00 PREC=0x00 TTL=64 ID=12028 DF PROTO=ICMP TYPE=8 CODE=0 ID=14 SEQ=592 
Nov  4 23:38:02 ubutower kernel: [ 1527.496012] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=22:7c:e3:ac:d3:c7:40:16:7e:84:26:87:08:00 SRC=192.168.0.210 DST=192.168.0.90 LEN=84 TOS=0x00 PREC=0x00 TTL=64 ID=12283 DF PROTO=ICMP TYPE=8 CODE=0 ID=14 SEQ=593 
Nov  4 23:38:03 ubutower kernel: [ 1528.254146] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=01:00:5e:00:00:fb:e6:79:fa:86:05:dd:08:00 SRC=192.168.0.62 DST=224.0.0.251 LEN=122 TOS=0x00 PREC=0x00 TTL=255 ID=54052 DF PROTO=UDP SPT=5353 DPT=5353 LEN=102 
Nov  4 23:38:03 ubutower kernel: [ 1528.322741] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=01:00:5e:00:00:fb:3c:8d:20:67:0e:8d:08:00 SRC=192.168.0.72 DST=224.0.0.251 LEN=393 TOS=0x00 PREC=0x00 TTL=255 ID=0 DF PROTO=UDP SPT=5353 DPT=5353 LEN=373 
Nov  4 23:38:03 ubutower kernel: [ 1528.323934] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=01:00:5e:00:00:fb:a0:6a:44:3c:49:c4:08:00 SRC=192.168.0.82 DST=224.0.0.251 LEN=388 TOS=0x00 PREC=0x00 TTL=255 ID=21352 DF PROTO=UDP SPT=5353 DPT=5353 LEN=368 
Nov  4 23:38:03 ubutower kernel: [ 1528.324911] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=01:00:5e:00:00:fb:b0:e4:d5:9d:70:62:08:00 SRC=192.168.0.56 DST=224.0.0.251 LEN=443 TOS=0x00 PREC=0x00 TTL=255 ID=38674 DF PROTO=UDP SPT=5353 DPT=5353 LEN=423 
Nov  4 23:38:03 ubutower kernel: [ 1528.325227] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=01:00:5e:00:00:fb:b0:e4:d5:9d:70:62:08:00 SRC=192.168.0.56 DST=224.0.0.251 LEN=395 TOS=0x00 PREC=0x00 TTL=255 ID=38675 DF PROTO=UDP SPT=5353 DPT=5353 LEN=375 
Nov  4 23:38:03 ubutower kernel: [ 1528.520012] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=22:7c:e3:ac:d3:c7:40:16:7e:84:26:87:08:00 SRC=192.168.0.210 DST=192.168.0.90 LEN=84 TOS=0x00 PREC=0x00 TTL=64 ID=12310 DF PROTO=ICMP TYPE=8 CODE=0 ID=14 SEQ=594 
Nov  4 23:38:04 ubutower kernel: [ 1529.544051] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=22:7c:e3:ac:d3:c7:40:16:7e:84:26:87:08:00 SRC=192.168.0.210 DST=192.168.0.90 LEN=84 TOS=0x00 PREC=0x00 TTL=64 ID=12389 DF PROTO=ICMP TYPE=8 CODE=0 ID=14 SEQ=595 
Nov  4 23:38:05 ubutower kernel: [ 1530.568016] IN=rosbridge OUT=rosbridge PHYSIN=eno1 PHYSOUT=tap0 MAC=22:7c:e3:ac:d3:c7:40:16:7e:84:26:87:08:00 SRC=192.168.0.210 DST=192.168.0.90 LEN=84 TOS=0x00 PREC=0x00 TTL=64 ID=12442 DF PROTO=ICMP TYPE=8 CODE=0 ID=14 SEQ=596
```
When I DOWN then UP the rosbridge on the cloud gateway, I see this log entry which I suspect is associated with DHCP:
```log
Nov  4 23:48:05 ubutower kernel: [ 2130.276607] IN=rosbridge OUT=rosbridge PHYSIN=tap0 PHYSOUT=eno1 MAC=ff:ff:ff:ff:ff:ff:22:7c:e3:ac:d3:c7:08:00 SRC=0.0.0.0 DST=255.255.255.255 LEN=322 TOS=0x00 PREC=0xC0 TTL=64 ID=0 PROTO=UDP SPT=68 DPT=67 LEN=302
```

So I am going to have to persist some iptables rules. How to do that?

Tried adding a script in /etc/network/if-pre-up.d, didn't work. Googling then testing shows  ifupdown not installed by default. Rather than try to install it, let me try networkd-dispatcher scripts.

I copied the same script to /etc/networkd-dispatcher/routable.d still no work. The only place that seems to work reliably is /etc/NetworkManager/dispatcher.d/pre-up.d

But does that work on the ubuntu minimal install on AWS? I tried the NetworkManager/dispatcher.d/pre-up.d script, did not work.

Testing ```systemctl --type=service``` both the desktop and the cloud gateway have networkd-dispatcher service active, so that approach should work. On the cloud machine, added a script to /etc/networkd-dispatcher/routable.d. Using ip link up/down did not fire, but ```netplan apply``` did! But the only interface I see is eth0, does not fire AFAICT for rosbridge or tap0. Well not yet, they are not 'routable' until I connect and DHCP fires.

I tried adding a ```renderer: networkd``` under the rosbridge netplan definition. That killed network configuration, had to reboot to restore it.

So far, nothing seems to work consistently. Let me try the iptables-persistent app

Desktop: sudo apt install iptables-persistent

I added a rule to DOCKER-USER, rebooted, and it persisted. Yay. How about on the AWS machine? Same drill, it also worked! So that's how I will do it.

Now how to get rid of the annoying startup script with ```apt install iptables-persistent```

I could not make it reappear to test. But the install suggests I try netfilter-persistent. I'm going to do a fresh AWS install, and try that. ... Fresh install does not have netfilter-persistent loaded. Install ```sudo apt install -y netfilter-persistent``` Yes installed, no annoying install prompt.

Test it:
```bash 
ubuntu@ip-172-31-23-190:/etc$ sudo apt install -y iptables
ubuntu@ip-172-31-23-190:~$ sudo iptables -N DOCKER-USER
ubuntu@ip-172-31-23-190:~$ sudo iptables -A DOCKER-USER -i rosbridge -o rosbridge -j ACCEPT
ubuntu@ip-172-31-23-190:~$ sudo iptables -L
Chain INPUT (policy ACCEPT)
target     prot opt source               destination         

Chain FORWARD (policy ACCEPT)
target     prot opt source               destination         

Chain OUTPUT (policy ACCEPT)
target     prot opt source               destination         

Chain DOCKER-USER (0 references)
target     prot opt source               destination         
ACCEPT     all  --  anywhere             anywhere            
ubuntu@ip-172-31-23-190:~$ sudo netfilter-persistent save
```
reboot. Did not work :( I see, /usr/share/netfilter-persistent/plugins.d has no plugins. Apparently the plugin is part of iptables-persistent. OK, try a suppressed install of iptables-persistent:
```bash
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y iptables-persistent
```
OK, no annoying prompt. Recreated DOCKER-USER, then:
```bash
sudo su
iptables-save > /etc/iptables/rules.v4
```
reboot. Worked.

Now back to the previous system that has openvpn configured. Connected to it, checked ping from remote system, started Docker, ping still works. Yay.

Now try DHCP. Disconnect. On cloud AWS, ```sudo netplan apply```. rosbridge has no IP as expected. Reconnect to OpenVPN. After a few seconds, rosbridge gets an IP address. Yay.

