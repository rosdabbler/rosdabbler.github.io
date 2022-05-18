---
title: "ROS2 package demo 9 - merging packages into a single repo"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-15
---
Been away from writing for a couple of weeks. In the first week, I dove deeper into rosdoc2. Ultimately I discovered that the existing rosdoc2 is a WIP. Proposed some changes, see [my fork](https://github.com/rkent/rosdoc2). I'd like to keep working on that, but for now there is a large existing pull request that needs to be landed for me to make further progress. So I'm going to move on for awhile.

I've learned that many packages are published as subdirectories of a larger repo. For example, [vision_opencv](https://github.com/ros-perception/vision_opencv) has *Packages for interfacing ROS with OpenCV* I want to modify my demo to be in that format. I've had though multiple attempts at this already. What am I going to do to reconcile all of this? I'm going to make *fqdemo* the repo with two packages, messages and nodes. Done.

After this merge, I want to update the workspace repo to be more correct with current config. Let me test what works. (Doing Terminal/Run Task...) in vscode.

* purge: yes
* build: yes (but I had to do a purge first)
* clean: No. I have inconsistent use of --merge-install. If fixed, adding --merge-install to clean.
* cpp_check: Failed, ament_cppcheck not found. What? Minimal documentation.
  * How to install? ```apt list | grep cppcheck``` showed that packages exist. Tried to install, ```sudo apt install ros-$ROS_DISTRO-ament-cppcheck``` but claims it is already installed. What gives? The task terminal is not sourcing the ros distro. Apparently shells under vscode tasks.json do not run .bashrc by default. Or so they claim, AFAICT only the path is messed up, other variables were there. To fix, I added options to commands that needed this, as follows:
  ```yaml
      {
        "label": "cppcheck",
        "detail": "Run static code checker cppcheck.",
        "type": "shell",
        "options": {
          "shell": {
            "args": ["-c", "-i"]
          }
        },
        "command": "ament_cppcheck src/",
      },
   ```
  * These also work now: flake8, fix, cppcheck, cpplint, import from workspace file, install dependencies, lint all, lint_cmake, pep257, uncrustify, xmllint.
  * new ament_cmake package works, but does adds it directly under src/. Same with new ament_python package.
  * pybuild works, but the new py package generated some warnings that need fixing.
  * test worked - but I have lots of issues to address.
  * update workspace file works.

