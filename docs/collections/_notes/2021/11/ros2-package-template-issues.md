---
title: "ROS2 package demo (issues) - notes on stuff to look into"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-24
---
- code coverage. See Code coverage in [geometric_shapes](https://github.com/ros-planning/geometric_shapes/blob/ros2/README.md)

- investigate using visibility file. See https://gcc.gnu.org/wiki/Visibility and [visibility_control.h](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/include/controller_manager/visibility_control.h) and https://answers.ros.org/question/361748/purpose-of-visibility_control-files-in-ros-packages/

- add a Quality Declaration like https://github.com/ros-tooling/libstatistics_collector/blob/galactic/QUALITY_DECLARATION.md

- get github credentials working in docker container

- How do I test private C++ methods without exporting their documentation?