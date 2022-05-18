---
title: "ROS2 package demo 7 - rosdoc2 and documentation"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-26
---
I've made some progress getting rosdoc2 to work with fqdemo, current issue is that the main node in fqdemo_nodes does not show up correctly. I've checked in fqdemo_nodes to show the issue. With today's commit **83575b60** here is how to run rosdoc2, and demo the issue. Run in the root of the fqdemo_nodes package directory:

```bash
rm -r doc/_build
rosdoc2 build -p ./ -d doc/_build/build -c doc/_build/cross_reference -o doc/_build/output
```

When I run, I get this error, which also shows up in the rendered documentation:
```
reading sources... [100%] index
/home/kent/ros2_wses/fqdemo_ws/src/fqdemo_nodes/doc/_build/build/fqdemo_nodes/package_name/default_sphinx_project/api/function_subpub__node_8cpp_1a0ddf1224851353fc92bfbff6f499fa97.rst:13: WARNING: doxygenfunction: Unable to resolve function "main" with arguments (int, char*) in doxygen xml output for project "fqdemo_nodes Doxygen Project" from directory: /home/kent/ros2_wses/fqdemo_ws/src/fqdemo_nodes/doc/_build/build/fqdemo_nodes/output_staging/generated/doxygen/xml.
Potential matches:
- def main(args = None)
looking for now-outdated files... none found
```
To see the rendered output, I open [this file](file:///home/kent/ros2_wses/fqdemo_ws/src/fqdemo_nodes/doc/_build/output/fqdemo_nodes/index.html) file:///home/kent/ros2_wses/fqdemo_ws/src/fqdemo_nodes/doc/_build/output/fqdemo_nodes/index.html

Solution: You have to define the main function in a .hpp file. When you do though, you get an error from spinx/breathe about duplicated functions. To fix this, you implement the DOXYGEN_SHOULD_SKIP_THIS fix that the dup error suggests.

