---
title: "ROS2 package demo 12 - Repo format"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-18
---

While trying to organize my workspace template, I see that most repos are organized with packages as the folders. I need to figure out if this is a standard or not.

I'll check [distribution.yaml](https://github.com/ros/rosdistro/blob/master/rolling/distribution.yaml) for a few multi-package repos. [ROS index](https://index.ros.org/repos/) is maybe a better source.

## Various package organizations:

* Mavros
  * separate repos for main, msgs, library.
  * main repo has same name as master repo.
  * the repo has a README and CONTRIBUTING file which is listed on index.ros.org page for the repo.

* [image_pipeline](https://github.com/ros-perception/image_pipeline)
  * msgs appear to be in separate repo,like ros-perception/vison_msgs
  * image_pipeline has a subdirectory image_piperline which does ament_package(). Not clear what that does. Not clear how meta works.

* geometry2
  * also has a geometry2 subdirectory.
  * See [this PR](https://github.com/ros2/geometry2/pull/184/files) for how the metapackage is created.

* ros2_controllers
  * Same thing, a ros2_controllers subdirectory for the meta package.

* ros2_control
  * Note the metapackage also has a ros2_control.repos file.
  * repo has a /doc directory with Doxyfile and index.rst

* rmf_battery
  * a single package, package is a subdirectory under repo. 
  * docs are at the repo level. This directory also includes a Makefile.

* cartographer_ros
  * subdirectory cartographer_ros contains nodes
  * separate cartographer_ros_msgs repo
  * documentation in /docs repo on repo, not package folders.

* ublox
  * subdirectory ublox is meta package.
  * messages in separate ublox_msgs folder.

* lanelet2
  * lanelet2 subdirectory is metapackage

Clearly rosdoc2 is intended as a package-level documentation generator. The repo level material is only appearing on index.ros.org, which is also the replacement for the wiki.

## Re-examine rosindex

I ran rosindex again, still hoping to be able to use the info there to do some basic scrapes. Took most of the day to run. 