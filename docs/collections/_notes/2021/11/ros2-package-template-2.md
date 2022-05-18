---
title: "ROS2 package demo 2 - repo design"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-17
---
## Separate repos, use a .repos file like ros2 itsells
f and ros2_control
Looking at major ros2 repos like ros2_control and ros2 itself, setup reads from a list of repos in a master repo. I want to organize in that matter.

The step [Get ROS 2 code](https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html#get-ros-2-code) has these instructions:
```bash
mkdir -p ~/ros2_eloquent/src
cd ~/ros2_eloquent
wget https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos
vcs import src < ros2.repos
```
I want to duplicate this setup. Created three github repos: a master **fqdemo**, subrepos **fqdemos-nodes** and **fqdemos-msgs**. Repo list:

```yaml
repositories:
  fqdemo-msgs:
    type: git
    url: https://github.com/rosdabbler/fqdemo-msgs.git
    version: main
  fqdemo-nodes:
    type: git
    url: https://github.com/rosdabbler/fqdemo-nodes.git
    version: main
```

I installed [Install development tools and ROS tools](https://docs.ros.org/en/eloquent/Installation/Linux-Development-Setup.html#install-development-tools-and-ros-tools)

apt failed, no python-rosdep. `apt list` shows me it is called python3-rosdep. Fixed, reran install, this time it worked.

Now follow instructions:
```bash
mkdir -p fqdemo-ws/src
cd fqdemo-ws
wget https://raw.githubusercontent.com/rosdabbler/fqdemo/main/fqdemo.repos
vcs import src < fqdemo.repos
```
Yuck, `vcs import` wants me to enter github login. Nope, I had an error in repo name. Worked and pushed to https://github.com/rosdabbler/fqdemo. (also fixed in printout above)

If I look at [ros2_control.repos](ros2_control.repos) I see that it is in a folder **ros2_control/ros2_control** with a bunch of other packages at the same level. **ros_control.repos** pulls in additional packages, sort of like subrepos under git. I think I will leave this as is for now.

I see that the checkout above was https:// I want to be able to modify, so the proper way is ssh. I see this **vcs** tool can creat the list automagically. I just need to clone repos first. Did that, ran `kent@ubutower:~/ros2_wses/fqdemo-ws$ vcs export src > fqdemos.repos and got for fqdemos.repos:
```yaml
repositories:
  fqdemo-msgs:
    type: git
    url: git@github.com:rosdabbler/fqdemo-msgs.git
    version: main
  fqdemo-nodes:
    type: git
    url: git@github.com:rosdabbler/fqdemo-nodes.git
    version: main
```
I'll leave the original for now.

### Organize workspace for vscode
[This](https://github.com/RoboGnome/VS_Code_ROS) reference has instructions.

When I emptied out the workspace, I got a "Clone Repository" message which sounded like a good thing to try. Also found with `Ctl-Shift-P Git: Clone` from the Git vscode extension.

I see I have misnamed these. Need an underscore, not a dash. Renaming everything.

From answers I also found [this](https://www.allisonthackston.com/articles/vscode-docker-ros2.html) which is more specific for ROS2. Using it for tasks.json

OK I have build tasks working, I think I will check it in.

### Debugging

Did a bunch of work, now can debug C++ in vscode! See checkins.

Here are full instructions starting from a bash prompt with ros2 sourced:
```bash
mkdir -p fqdemo-ws/src
cd fqdemo-ws
wget https://raw.githubusercontent.com/rosdabbler/fqdemo/main/fqdemo_ssh.repos
vcs import src < fqdemo_ssh.repos
colcon build
source install/setup.bash
code
# Add to workspace these folders in order: src/fqdemo-nodes, src/fqdemo-msgs, ./
# save as ./fqdemo.code-workspace and exit
code fqdemo.code-workspace
```
In code:
- set a breakpoint at fqdemo_nodes/src/demo_sub_pub.cpp at `rclcpp::init(argc, argv);`
- open "RUN AND DEBUG" from side icon
- In run select '(gdb) subpub debug(fqdemo_nodes)

Hmmm, did not work. I bet I did not compile with debug symbols? Retried:
```bash
rm -r build install log
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'
# etc.
```
This time it worked (that is, it stopped on the breakpoint unlike before). So it is important to add debug symbols to the build.
