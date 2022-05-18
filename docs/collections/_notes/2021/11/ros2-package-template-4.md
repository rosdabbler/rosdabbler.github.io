---
title: "ROS2 package demo 4 - python node with tests"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-19
---
I'm going to write the same basic node in python, and add tests.

## Python subpub node
Following generally the design of the [simple publisher and subscriber (Python)](https://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-python) demo, I'll rewrite the C++ subpub node into python.

First thing I note is that intellisense is not finding my packages. [This question](https://answers.ros.org/question/366598/how-can-i-make-vs-code-recognize-ros2-python-packages/) led me to the [Microsoft ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros). Hey it worked, that was easy!

Then I tried adding the python stuff to my existing C++ package. `colcon build` but no python executables generated. Googling around led me [here](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/) with instructions for a combined C++ / python package. VERY different! With those changes, got a python node working.

### Testing
Now on to testing. `colcon test` now delivers new errors from linting the python code. Boo, they want 4 space indent. Fixed all the lint errors, checked it in.

I want to add a my own test. Googling gives conflicting info. Like [this post](https://www.theconstructsim.com/create-python-publisher-ros2/) wants me to add to package.xml:
```xml
  <!-- These test dependencies are optional
  Their purpose is to make sure that the code passes the linters -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
```
but the linters are running without that. I see tests are running, let me just add mine to the directory.

Tried that, does not work. Fixed linting, but `assert False` never fails. If I just run `pytest` in the test directory it fails as it is supposed to. So whatever is running the linting tests is not running other tests. I'll try the suggestions above.

Nope. Searching through ros packages, I see an example like:
```bash
if(BUILD_TESTING)
  find_package(ros_testing)

  add_ros_test(iceoryx_integrationtest/test_roudi_startup_shutdown.py)
  add_ros_test(iceoryx_integrationtest/test_callback_example.py)
```
so it looks like that is the ticket? Yes, plus a few more config options copied from various examples. Anyway got a failing test to fail, now to add a real test. Tried, failed missing "generate_test_descrption".

More searching, rostest is supposedly for integration tests. Found [ament_cmake_pytest](https://github.com/ament/ament_cmake/blob/master/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake) which looks to be more appropriate for unit tests. Trying that.

OK that worked. Checked that in.

Tested with:
```bash
ros2 topic pub /num_power fqdemo_msgs/msg/NumPwrData "{num: 3, power: 2}"
```
That showed a run error (which makes it clear I need a full node test). Fixed, checked in.

## Documentation

I'm going to start working on documentation, both actually doing it, but also adding the tooling to generate the documentation.


### ROS REP-2004 Quality Guide
Came across [REP-2004](https://www.ros.org/reps/rep-2004.html) today which is about quality goals for ros2. I should use that as a standard. Here's the relevant section:

>3.: Documentation:
>
>3.i: Must have documentation for each "feature" (e.g. for `rclcpp`: create a node, publish a message, spin, etc.)
>
>3.ii: Must have documentation for each item in the public API (e.g. functions, classes, etc.)
>
>3.iii: Must have a declared license or set of licenses
>
>3.iv: Must state copyrights within the project and attribute all authors
>
>3.v: Must have a "quality declaration" document, which declares the quality level and justifies how the package meets each of the requirements
>
>    3.v.a: *Must have a section in the repository's ``README`` which contains the "quality declaration" or links to it*
>
>    3.v.b: *Should register with a centralized list of 'Level N' packages, if one exists, to allow for peer review of the claim*
>
>    3.v.c: *Must reference any 'Level N' lists the package belongs to, and/or any other peer review processes undergone*

This document has no standards at all pertaining to the code itself, unless that code is part of the "public API" where 3.ii applies. But it seems like the "API" documentation that exists, particularly for C++, is just a parsing of the C++ code base.

### Doxygen CI
If I used Doxygen to generate the documentation, what tooling exists for that. [This post](https://joeloskarsson.github.io/2018/automatic-docs) proposes a way using Travis and Github pages. There's also [this solution](https://stackoverflow.com/questions/36064976/using-doxygen-in-read-the-docs) using readthedocs. At first glance that is what I would prefer. That in turn references [this great blog post](https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake/). Seems like exactly the point of **fqdemo** to look into stuff like this!

But in any case, for step is getting the code to a point where the documentation exists and can be generated using DOxygen.

### Demo of Doxygen

I started the [Doxygen getting started](https://www.doxygen.nl/manual/starting.html) tutorial at ~/projects/doxydemo. No, on second thought I'll go directly to fqdemo_nodes.

I created a Doxygen config file, per the manual: `doxygen -g`  That generates a very large file with zillions of options. **ros_control** uses that base file, so I compared me to them, and adjusted my file to match. Now running works:
```bash
kent@ubutower:~/ros2_wses/fqdemo_ws/src/fqdemo_nodes$ doxygen Doxyfile 
```
But I need to show what changes. To do that, I'm going to checkin the original file, then show my changes.