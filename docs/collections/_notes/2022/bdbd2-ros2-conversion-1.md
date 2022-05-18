---
title: "BDBD ROS2 conversion 1"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2022-01-07
---
Today I created a new repo bdbd2 and a new ROS2 workspace bdbd2_ws to begin converting my bdbd code to ROS2. I'm going to follow all of the lessons that I learned during work on fqdemo.

# Documentation Boilerplate

First, I'll create standard ROS2 files. I'll have three packatges: messages, meta, and nodes. This is intended as support for the hardware on my modified Jetbot Nano-based robot.

## Create the basic packages, and init the workspace:
```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bdbd2_msgs
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bdbd2_nodes
```
I also created a package for bdbd2. The folder hierarchy is:
```bash
bdbd2_ws/
  src/
    bdbd2/
      package.xml
      bdbd2_msgs/
        package.xml
      bdbd2_nodes/
        package.xml
```

Update each package.xml with description, author, urls.

Add CHANGELOG.rst, README.md, and CONTRIBUTING.md in each package (they already have LICENSE, package.xml, and CMakeLists.txt from `ros2 package create`

## Problems with my metadata format

`colcon build` is not finding my subpackages. I traced the issue: in `github/colcon/colcon-recursive-crawl/colcon_recursive_crawl/package_discovery/recursive_crawl.py' we have this code:
```python
                try:
                    result = identify(identification_extensions, dirpath)
                except IgnoreLocationException:
                    del dirnames[:]
                    continue
                if result:
                    descs.add(result)
                    del dirnames[:]
                    continue
```
If I change this to this it works:
```python
                try:
                    result = identify(identification_extensions, dirpath)
                except IgnoreLocationException:
                    del dirnames[:]
                    continue
                if result:
                    descs.add(result)
                    # del dirnames[:]
                    continue
```
I can also make it work by adding an option `--base-paths . ./*/*` to colcon commands. So my conclusion is, this is not working. I don't think that is a reasonable approach.

So I am going to revise fqdemo.
