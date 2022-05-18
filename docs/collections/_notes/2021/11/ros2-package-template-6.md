---
title: "ROS2 package demo 5 - more on documentation"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-23
---
I'm hoping today to get a basic package documentation done, with automatic generation using github actions.

# Issues with building

Unfortunately I tried using the builds from the tasks, which are `colcon build --merge-install --symlink-install ...` but these do not work when I try to source the workspace outside of Docker. Apparently the symbolic links use symbolic links to filenames that are not valid outside of Docker, like:
```bash
ros@ubutower:/workspaces/fqdemo_ws/install/share/fqdemo_nodes$ ls -l local_setup.bash
lrwxrwxrwx 1 ros ros 87 Nov 24 18:48 local_setup.bash -> /workspaces/fqdemo_ws/build/fqdemo_nodes/ament_cmake_environment_hooks/local_setup.bash
```
When I try to run `. install/setup.bash` outside of the Docker container, this fails.

In [VSCode, Docker, and ROS2](https://www.allisonthackston.com/articles/vscode-docker-ros2.html) I see that Allison does **not** specify the --symlink-install option. I think I need to remove that by default, and only use it for special python development.

# Documenting the packages
I added some documentation to fqdemo-nodes, and I generally get something useful when I run at the package level: `doxygen doc/Doxyfile. Tried the same thing with fqdemo-msgs, I get nothing. Need to figure that out.

I just discovered https://github.com/ros-infrastructure/rosdoc2 which looks promising.

rosdoc2 says to install, run `pip install -U rosdoc2`. But pip can't find it, neither can Pypi.

Tried installing from source. I cloned the repo, and ran:
```bash
$ python3 setup.py build
$ sudo python3 setup.py install
```
Now I have it. Ran it, it worked somewhat on fqdemo_nodes, not on fqdemo_msgs.