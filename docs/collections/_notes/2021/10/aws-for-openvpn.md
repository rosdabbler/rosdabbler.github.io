---
title: "Setting up an Amazon Web Services (AWS) virtual machine usable by OpenVPS"
layout: archive
author_profile: true
tags: [aws, networking]
date: 2021-10-26
---

Here I'll describe how to setup an simple virtual machine using AWS, configured to be usable as an OpenVPN gateway.
These instructions are designed to work on Ubuntu, but should work on other Linux-based systems as well.

I assume basic Linux and networking knowledge. Also these instructions are to launch a demo machine you intend to
throw away, for permanent use you may want tighter security.

## Get an Amazon Web Services account

Open the [AWS Management console](https://aws.amazon.com/console/), create an account and login to the console.

Select **EC2** from **Services**.

## Get and configure an IP address for the virtual machine

Select **Elastic IPs**

Select **Allocate Elastic IP Address**

Select **Allocate**

After you have the IP address, the easiest way to use it will be to add this to the hosts file on your computer.
That is, as root edit **/etc/hosts** and add a line for the ros-openvpn host. That is, I got the IP address
**44.232.53.80** which I added to **/etc/hosts** as:

```
44.232.53.80 ros-openvpn
```

## Create and configure a Key Pair

From the AWS EC2 console, select **Key Pairs**

Select **Create Key Pair**

Enter a name, in this example I'll use **ros-openvpn**. Accept the defaults (**RSA** type, **.pem** format)

Select **Create key pair**

At this point, you will be prompted to download a file with the key file. Save this file and move it
to the directory ~/.ssh

Tighten the security on the .pem file:

```bash
cd ~/.ssh
chmod 400 ros-openvpn.pem
```

Edit the file ~/.ssh/config and add the lines:
```config
Host ros-openvpn
  HostName ros-openvpn
  IdentityFile ~/.ssh/ros-openvpn.pem
  User ubuntu
  StrictHostKeyChecking no
```
I used the **StrictHostKeyChecking no** option because I created and destroyed many AWS EC2 instances while testing stuff, so this lets me login to a new system with old credentials and the same IP address. Not recommended for a permanent system.

## Create a security group with SSH and OpenVPN access
From the EC2 console, select **Security Groups**

Select **Create security group**

Set the **Security group name** to ```openvpn-gateway```

Set the **Description** to ```SSH OpenVPN Ping```

Add Inbound rules for UDP port 1194, SSH (TCP port 22), and ICMP Echo Request, each with Anywhere-IPv4 Source.

Select **Create security group**

When finished, the Inbound rules for the security group should look like:

![Inbound Rules](/assets/images/AWS-openvpn-security-group.png)

## Launch an instance
From the EC2 console, select **Instances**

Select **Launch Instances**

At **Step 1: Choose an Amazon Machine Image (AMI)** you'll need to find an appropriate image from a massive list. We
are looking for a recent official minimal Ubuntu image, version 20.04. Here's how I have found to search for the correct
image.

Select **Community AMIs**

In the search field, enter ```Canonical Minimal amd64 20.04 /images/ YYYYMM``` where **YYYYMM** is the current year and month.
At the time this is written, that is 202110 so the full search field is:

```
Canonical Minimal amd64 20.04 /images/ 202110
```

Click **Select** for the most recent of these images (though any should work).

Click **Review and Launch** which brings you to **Step 7: Review Instance Launch**.

By **Security Groups** select **Edit Security Groups**.
Under **Assign a security group**, select **Select an existing security group** and select the security group we
created previously (openvpn-gateway).

Select **Review and Launch** then **Launch**

On the popup screen **Select an existing keypair or create a new key pair** make sure that **Choose an existing key pair**
is selected, with **Select a key pair** set to the key pair we created previously (called ros-openvpn).
Select **I acknowledge ...** then **Launch Instances**.

Select **View Instances**

## Assign the allocated Elastic IP to the instance

At the EC2 console, select **Elastic IPs**  Select the Ip we allocated, then under **Actions** select **Associate IP Address**.
There accept the default **Resource type** of **Instance**, select the search box with the *Choose and instance* text, and
select the now-running instance that we just launched.
Select **Allow this Elastic IP address to be reassociated**, then **Associate**.

If all works as expected, you should be able to ping the new instance:

```bash
$ ping ros-openvpn
PING ros-openvpn (44.232.53.80) 56(84) bytes of data.
64 bytes from ros-openvpn (44.232.53.80): icmp_seq=1 ttl=31 time=11.7 ms
64 bytes from ros-openvpn (44.232.53.80): icmp_seq=2 ttl=31 time=11.7 ms
64 bytes from ros-openvpn (44.232.53.80): icmp_seq=3 ttl=31 time=11.3 ms
64 bytes from ros-openvpn (44.232.53.80): icmp_seq=4 ttl=31 time=11.3 ms
```

You should also be able to ssh to the instance:
```bash
$ ssh ros-openvpn
Welcome to Ubuntu 20.04.3 LTS (GNU/Linux 5.11.0-1020-aws x86_64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage


This system has been minimized by removing packages and content that are
not required on a system that users do not log into.

To restore this content, you can run the 'unminimize' command.

0 updates can be applied immediately.

Last login: Sun Oct 24 21:41:19 2021 from 50.35.65.128
To run a command as administrator (user "root"), use "sudo <command>".
See "man sudo_root" for details.

ubuntu@ip-172-31-17-172:~$ 
```
There may be ssh error messages about host changed, but the **StrictHostKeyChecking no** option
we recommended should allow login regardless.

You know have a gateway that can be used to test ROS networking with AWS nodes.
