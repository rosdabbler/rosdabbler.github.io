---
title: "ROS2 package demo 3 - lint and gtest"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-18
---
The goal for today is to get linting working on the repo, then divide the repo into .cpp and .hpp and add some documentation.

### Linting

Hey, a [really great post](https://answers.ros.org/question/306194/ros1-and-ros2-linting/) about this on ROS answers!

I really need a way to search over some core ROS2 code for examples. Let me clone a few repos and add them to a VS Code workspace. Which ones?
- ros2
- rclcpp
- common_interfaces
- rcl_logging
- geometry2
- message_filters
- ros_testing
- moveit2
- ros-controls/ros2_controls

Searching for "if(BUILD_TESTING)" shows lots of variants. "ament_lint_auto" is common. Checking [its docs](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst) it seems pretty straightforward. I'll try it.

Hmmm, seem to already have this in **CMakeLists.txt** How about package.xml? Yes. So I can run `colcon test`. Do it, shows running test, output shows "with test failures" but noe listed. What gives? Where is stderr? [This colcon issue](https://github.com/colcon/colcon-output/issues/15) gave me this suggestion which works: `colcon test-result --verbose` But why isn't that in the log? ... OK I see, I was looking in the wrong directory

So I did not have to make any changes, and I am getting lint errors! Trying to fix them.

One problem I cannot fix: it wants the header hard to be FQDEMO__DUMMY_HPP_ but I think it should me FQDEMO_NODES__DUMMY_HPP_ Why? check is done in ament_cpplint.main, going to modify that and see if I can figure out why. OK I see, wrong subdirectory name: include/fqdemo instead of include/fqdemo_nodes

Success!

### separate .hpp

First, I have to make a legit .hpp for the file.  I need something with a node. Looking at https://github.com/ros2/demos/blob/master/logging_demo/include/logging_demo/logger_usage_component.hpp then some others. Amazingly hard to find a simple node example. Finally pieced it together and pushed the changes.

### gtest

**sensor_msgs** pack kage has a simple implementation of this. I'll use that as an example and implement. No, that had no node. Took a lot of searching to figure it out, kep getting linking problems. I ended up make a project library that is used by the test.

Anyway, now I have a successfully testing repo with lint and gtest. Checking in.

### Next steps

I'd like to do an integration test to confirm node message processing. See [here](https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py) for how to do that.
