---
title: "BDBD ROS2 conversion 2"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2022-01-18
---
I've been away for over a week due to a sudden trip to California with a family emergency, but I am back now. But where an I at? Looks like I an getting the pan/tilt mechanism working in the new bdbd2 repo.

I'd moved PCA9685.py, the Jetbot driver for pan/tilt, to bdbd2. Now I need to fully integrate this into docker and ROS2 dependencies.

## Local Download of ROS repos

But first, I really need to ability to search ROS repos for code. I'm going to do a local download of all repos.

[This github](https://github.com/ros/rosdistro) contains the list of repos. [vcstool](https://github.com/dirk-thomas/vcstool) is how you download things. I already have it installed locally as `vcs`.

Yuck, `vcs` does not work directly with the rosdistro format. I need to do a conversion. I'll write a little script to do that.

### Distro to vcs script

I made a little script at /srv/rosrepos to convert rosdistro files to fils usable by vcstool:
```python
#!/usr/bin/python
"""Convert a rosdistro yaml file into one usable by vcstool"""
import yaml

def main():
    print('dtov')
    inpath = '/home/kent/github/ros/rosdistro/melodic/distribution.yaml'
    outpath = 'melodic.repos'

    with open(inpath, 'r') as file:
        rosdistro = yaml.safe_load(file)

    repositories = {}

    for key, value in rosdistro['repositories'].items():
        print(f'repo: {key}')
        if 'source' in value:
            repositories[key] = value['source']

    repos = {}
    repos['repositories'] = repositories
    with open(outpath, 'w') as file:
        yaml.dump(repos, file)

if __name__ == '__main__':
    main()
```
Just run this, modifying the code to get the distro that you want. After running, you can download all by running:
```bash
mkdir melodic
vcs import melodic < melodic.repos
```

## PCA9685 python for ROS2 repo.

I modified the PCA9685 in bdbd2 repos to run a simple test when called. Now I'll run it under Docker, including adding the install info.

I did a bunch of work on permissions to get i2c available on the jetbot under Docker. Working now.

This snippet gets a group's ID, but I never actually used it:
```bash
declare $(getent group i2c | awk -F: '{printf "I2CGROUP=%d\n", $3}')
```

Now for dependencies. The names for dependencies is listed [here](https://github.com/ros/rosdistro/tree/master/rosdep). I added python-smbus based on that.

Here is how to install dependencies:
```bash
kent@nano:~/bdconv_ws/bdbd2_ws $ rosdep install --from-paths src --ignore-src --rosdistro rolling -r -y
```
I need to run this automatically as part of a startup of the Dockerfile, but so far I have not figured this out yet.

Figured it out - but it does not work because python-smbus is not a valid debian package anymore. I ended up fixing the Dockerfile. But that file is not in any repo.

Next step: Add the workspace itself as a repo so that I can start to collect the changes I am making to the Docker environment. Done, now I have rkent/bdbd2_ws.

2022-01-19

## PanTilt node

Created drivers, got it working with pantilt.

## Documentation

The documentation was not working. Fixed what i could, but it appears that I have lost some my previous work on rosdoc2. In particular, I cannot find where Standard Documents are being generated. Bummer, I suppose I need to redo that. Maybe I never did it, and the generated files were created manually? Not sure, but starting to believe that.

So I am not going to return to rosdoc2 after all, at least until more progress is made upstream.

2022-01-20

I got motors working. Going to switch to audio.

## Audio tests

audio.py still works on original bdbd. Trying in Docker.

I installed espeak-ng on container. On base system, this works:

```bash
espeak-ng "This is a test"
```
Does not work in container. My mission is to figure out why.

After much messing, I got SOMETHING to work. With every possible thing, this will play:
```bash
aplay --device plughw:CARD=ArrayUAC10,DEV=0 /usr/share/sounds/alsa/Front_Center.wav
```
I added that to a file "atest.sh" and now I'll try to see what I really need.

- added user ros to audio group, now user ros works.
- removed --privileged, still works
- removed "--device=/dev/snd" does not work
- removed "--device=/dev/bus" still works

Now I'm trying to get the default device set. This [link](https://www.volkerschatz.com/noise/alsa.html) is helpful for config. This /etc/asound.conf works but geberated errors on startup:
```
pcm.!default {
  type plug
  slave {
    pcm "hw:2,0"
  }
}
```

Some of the errors went away when I added back "--device=/dev/bus" so I think I will leave that. Now the only error is: "aplay: main:852: audio open error: No such file or directory"

Now espeak-ng still does not work, but this works:

```bash
espeak-ng -d default:CARD=ArrayUAC10 "Hello"
```

espeak-ng implies it needs pulse audio, but that is not installed. But I cannot seem to configure anything without it. Tried installing, no help (claims not running).

OK now I've got it! I think I need to do all of this:

1. Run docker with:
```
    "--device=/dev/bus",
    "--device=/dev/snd",
```
2. I have all of this in the Dockerfile:
```Dockerfile
# Respeaker firmware
RUN pip install py-espeak-ng
RUN pip install pyusb click
RUN git clone https://github.com/respeaker/usb_4_mic_array.git /home/ros/respeaker

# bash additions
COPY bashrc_add.sh /tmp/
RUN cat /tmp/bashrc_add.sh >> /home/ros/.bashrc

RUN apt-get install -y \
  usbutils \
  pulseaudio

COPY 60-respeaker.rules /etc/udev/rules.d
COPY pulse.conf /tmp/pulse.conf
COPY asound.conf /etc/
RUN cat /tmp/pulse.conf >> /etc/pulse/default.pa
RUN usermod -aG audio ros
```
Now this works:
```bash
espeak-ng "Hello"
```

I'm going to remove stuff now to check what I really need.

I've shortened it to this:
```Dockerfile
# Respeaker firmware
#RUN pip install py-espeak-ng
#RUN pip install pyusb click
#RUN git clone https://github.com/respeaker/usb_4_mic_array.git /home/ros/respeaker

# bash additions
COPY bashrc_add.sh /tmp/
RUN cat /tmp/bashrc_add.sh >> /home/ros/.bashrc

#RUN apt-get install -y \
#  usbutils \
#  pulseaudio

#COPY 60-respeaker.rules /etc/udev/rules.d
COPY asound.conf /etc/
#COPY pulse.conf /tmp/pulse.conf
#RUN cat /tmp/pulse.conf >> /etc/pulse/default.pa
RUN usermod -aG audio ros
```
I'm going to commit it like this in case problems crop up again later.

2022-01-21

Working on sayit node. That was easy.

Now working with jetbot_ros to try to get the camera up.

It all runs in Docker, which makes me realize I don't know how to run multiple ROS2 Docker containers, and have Ros find everything.

So let me return to first principles with the camera.

This code works: https://github.com/JetsonHacksNano/CSI-Camera/blob/master/simple_camera.py and it seems to use display 0.

I'm checking out suggestions in https://forums.developer.nvidia.com/t/how-to-use-mipi-csi-camera-inside-docker-container/179439 for running the camera under docker.

Yuck I did an upgrade on nano, and now my docker is broken. I also almost ran out of disk space today, so I bought a larger sdcard (256 GB) and am currently installing it.

## Reinstall on large SD card

Got the image from https://developer.nvidia.com/jetson-nano-sd-card-image as jetson-nano-jp46-sd-card-image.zip, installed on a 256Gb micro sd card.

Setup:

- apt update && app upgrade
- added kent to docker group
- copied .ssh from ubutower
- installed nano
- installed Firefox
- installed pip `sudo apt install python-pip`
- installed jtop `sudo pip install jetson-stats`
- installed htop
- installed vscode

2022-01-22

Tried to clone rkent/bdbd, did not work. I'm missing some github authentification step. Maybe ssh-add? No, added this to ~/.ssh/config:
```
Host github.com
  HostName github.com
  IdentityFile ~/.ssh/github_rsa
  IdentitiesOnly yes
```

I also cloned waveshare/jetbot. Tried to install, missing setup tools.
```bash
sudo apt install python3-pip
sudo pip3 install
cd ~/github/waveshare/jetbot
sudo python3 setup.py install
```
That generated a lot of errors, I assume because I am using the newest nvidia build? Some errors were pip related. System pip is really old, tried updating but this causes new issues. Recommended is to just upgrade user pip using:
```bash
python3 -m pip install --upgrade pip
```
I tried running waveshare jetbot stats, no luck. Next I tried bdbd stats, missing module. Fixed with:
```bash
sudo -H python -m pip install Adafruit_SSD1306
```
Now missing jetbot.utils.utils. Grrr. I think I'm going to try the nvidia jetbot instead from https://github.com/NVIDIA-AI-IOT/jetbot

Did that, now missing traitlets. Installed that. Etc for more missing stuff.

Notice also that the name of the service jetson_stats installed by jetbot seems to conflict with the name used for the service that jtop uses.

Frrr, still having problems. It insists on looking for tensorrt which I do not need. I modified stats.py to not require all of this.

```
pip install Adafruit_ADS1x15
```
the good news is, if I bypass all of the errors, then the display works. Just need to fix the underlying code.

While debugging, I modified:
- /usr/local/lib/python2.7/dist-packages/Adafruit_PureIO/smbus.py

It seems that the Adafruit ADS1x15.py library is obsolete. I see waveshare has its own library ADS1115.py that I should try. It has a self-test. Needed ```python3 -m pip install smbus``` but after that it worked.

I modified bdbd stats.py to incorporate the Waveshare battery read. Now the display works, on both python 2 and 3. But you need to install smbus, Pillow, and others.

The bad news is the battery is busted again. Probably means the charger is broke, not just the one cell :(
