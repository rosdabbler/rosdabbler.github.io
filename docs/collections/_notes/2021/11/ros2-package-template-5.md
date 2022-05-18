---
title: "ROS2 package demo 5 - filling in documentation"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-23
---
Last time, I got an initial Doxygen config working. In the meantime, I found various ways to beautify Doxygen.

## VSCode ROS2 Workspace Template
I also reviewed and am impressed by the [VSCode ROS2 Workspace Template](https://github.com/athackst/vscode_ros2_workspace) which has very similar goals to what I am working on here. I'm going to try that, and consider moving my demo to a fork of that workspace.

That repo is designed to run in Docker. OK, I'll try that feature.

### ROS2 distribution

Config files also point to foxy. No, I wanted later. Might as well switch to **rolling** now. Installing.

The althack/ros2 images on hub.docer.com do not support rolling. I pulled the underlying [Dockerfile](https://github.com/athackst/dockerfiles/blob/main/ros2/galactic.Dockerfile) for galactic, and modified it to support rolling. Checked in to https://github.com/rosdabbler/vscode_ros2_workspace branch rolling.

### Documentation support

That repo template has support for documentation generation. I need to understand what it is doing, and see if that is the path that I want. Looking that over, [Mkdocs Simple Plugin](https://athackst.github.io/mkdocs-simple-plugin/mkdocs_simple_plugin/plugin/) claims to make documentation from source files - but I have none. So change paths, let's instead redo fqdemo using this repo as a base.

### Adaptation to fqdemo

So how do I do this? Looking over the template, this workspace is designed to replace fqdemo. I'm going to start a new repo, which I'll call fqdemo-ws, using **VSCode ROS2 Workspace Template** as a starting point.

OK done, changed rep to rolling, added fqdemo subrepos, commit and push to branch rolling. Let me see what works or not.

### Finding C++ includes
This line in demo_sub_pub.cpp has the red squiggle "not found":
```cpp
#include "fqdemo_nodes/demo_sub_pub.hpp"
```
I see that is also failing in old ws, I think I solved it by sourcing the workspace install before I opened VSCODE.

After some playing, it seemed to start working. I think you have to build the workspace for it to work.

### Debug
Trying debug on C++. Seems to work.

Trying debug on Python. I modified PySubPub.py to run as main. Now I can run it locally, but when I try to run under the debugger I get:
```
Exception has occurred: RCLError
Failed to create publisher: type_support is null, at /tmp/binarydeb/ros-rolling-rmw-cyclonedds-cpp-1.0.0/src/rmw_node.cpp:2007, at /tmp/binarydeb/ros-rolling-rcl-4.0.0/src/rcl/publisher.c:116
  File "/workspaces/fqdemo_ws/src/fqdemo_nodes/fqdemo_nodes/PySubPub.py", line 31, in __init__
    self.publisher = self.create_publisher(NumPwrResult, 'power_result', 10)
  File "/workspaces/fqdemo_ws/src/fqdemo_nodes/fqdemo_nodes/PySubPub.py", line 62, in main
    pySubPub = PySubPub()
  File "/workspaces/fqdemo_ws/src/fqdemo_nodes/fqdemo_nodes/PySubPub.py", line 68, in <module>
    main()
```
I'm going to pursue this later, for now I would rather work on documentation issues.

### Documentation
I see that she wrote Mkdocs Simple Plugin which extracts comments from source files. That is a fairly narrow choice, with Doxygen being much more common (and presumably robust). I think I'll focus there instead.

[rosdoc_lite](http://wiki.ros.org/rosdoc_lite) talks about how to document ROS packages - but is not going to be updated for ROS2.

[Per Package Documentation Proposal](https://github.com/ros2/design/pull/301) is discussed in that PR. Also https://github.com/ros2/design/blob/gh-pages/articles/per_package_documentation.md 

>1.5 Package documentation must be written and formatted in rst (reStructuredText)
>
>Rst is the file format currently utilized by the ROS 2 documentation. Package documentation should continue to utilize rst for consistency.
>
>Doxygen will produce API docs for any C++ code it is run on, and the maintainer has the option to further elaborate on their API docs by including Doxygen comment blocks within their source code.
>
>Once Doxygen is completed, or if it never ran because the tool did not find any doxyfile or C++ code, the tool will proceed to running Breathe and Sphinx.
>
>Breathe imports the symbols from Doxygen’s output to Sphinx. It will allow C++ API docs to be built by Sphinx to maintain consistency across ROS 2 API docs and allow cross-referencing between all packages.

So it looks like they are pretty set on .rst for package documentation. I guess I am going to have to learn that. But for now, let me do what I think is a decent job of documenting the source using Doxygen.