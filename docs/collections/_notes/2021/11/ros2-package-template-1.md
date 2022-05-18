---
title: "ROS2 package demo 1 - cpp tutorials"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-11-16
---
As I envision moving bdbd for ROS1 to ROS2, I want to start from the beginning with development practices that meet full current [ROS2 developer guide](https://docs.ros.org/en/galactic/Contributing/Developer-Guide.html) That includes:
- full use of package.xml
- cpp-centric
- code style guidelines
- linting
- automated tests (See ros2_control for using Github actions)
- heavy use of Docker
- standard internal documentation
- use of formatter (uncrustify or clang-format) (See [ros2_control commit](https://github.com/ros-controls/ros2_control/pull/491/commits/afdc0ea1252c60b25e49b0ebc830c9883e87f8c1) on this.

I'll start from the [ROS2 Getting Started tutorials](https://docs.ros.org/en/galactic/Tutorials.html#beginner-client-libraries "Beginner Client Libraries") and work my way into the quality guide for a complete but basic package.

## Creating a Workspace
Using [this](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html) tutorial starting [here](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#create-a-new-directory).

"2 Create a new directory" ... "3 Clone a sample repo" ... "4 Resolve dependencies" ... "5 Build the workspace with colcon" ... "6 Source the overlay" OK nothing new here. Moving on.

## "Creating your first ROS 2 package" tutorial
[Creating your first ROS 2 package](https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html).

"1 Create a package" ... "2 Build a package" ... "3 Source the setup file" ... "4 Use the package" ... "5 Examine package contents"

Here is start using VS Code with the workspace. Get messages "CMakeLists.txt was not found in the root of the folder 'fqtemplate'" (which is my workspace) from CMake Tools (extension). tried clicking "Locate" then selected the top-level CMakeLists.txt for the new package. Switched to [the extension Get Started page](https://code.visualstudio.com/docs/cpp/CMake-linux) to see what to do. Chose the kit "GCC for x86_64-linux-gnu 9.3.0". Now select a variant:
> To select a variant, open the Command Palette (Ctrl+Shift+P) run the CMake: Select Variant command.

I selected Debug. Tried Configure:
> Now that you've selected a kit and a variant, open the Command Palette (Ctrl+Shift+P) and run the CMake: Configure command to configure your project. 

Tried to build: "select the Build button from the Status bar." None of this seems to work AFAICT. I'm going to disable that extension for now. ... Done, but now I have no syntax highlighting with CMakeLists.txt. [This guy](https://github.com/athackst/vscode_ros2_workspace) shows he uses that extension with ROS2. I'll re-enable it. Not making any progress with this, so I'll move on for now. Maybe later I'll figure it out. Moving on.

## Writing a simple publisher and subscriber (C++)

[Next tutorial](https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). 

"1 Create a package" ... "2 Write the publisher node".

Checking out "std::chrono_literals", see [this](https://en.cppreference.com/w/cpp/symbol_index/chrono_literals). This code is using a "_" suffix to indicate class variables. Is that a ROS standard? I see it is a [Google C++ standard](https://google.github.io/styleguide/cppguide.html#General_Naming_Rules):
> Data members of classes (but not structs) additionally have trailing underscores.

Moving on to "2.2 Add dependencies" I see that my email address and name were already filled in, but name is "kent". Where does it get that from? I see [this question](https://answers.ros.org/question/204444/is-there-a-way-to-set-defaults-for-packagexml-maintainer-name/) but the answers do not say it is possible - yet clearly it is happening. [This issue](https://github.com/ros2/ros2cli/issues/477 "Default email when creating a package should use local user.email value, not global") seems relevant. Looking at the code link from that, I see that the default is set using python getpass.getuser(), which states:

>  getpass.getuser(): Return the “login name” of the user.<br><br>
>  This function checks the environment variables LOGNAME, USER, LNAME and USERNAME, in order, and returns the value of the first one which is set to a non-empty string. If none are set, the login name from the password database is returned on systems which support the pwd module, otherwise, an exception is raised.

I tried setting LOGNAME and then ran that python code, returned name matches. Setting that in my .bashrc. Tested with ```ros2 pkg create``` and it works.

Anyway, continued fixup of packages.xml & CMakeLists.txt, build and ran OK. For now I do not need to subscriber.

## Filesystem

[Filesystem layout'](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#filesystem-layout) shows a number of things that are missing from this tutorial. I'm going to add them all [from here](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html#package-layout), and try to populate with something in a demo. I now have the package defined as so:
```bash
kent@ubutower:~/ros2_wses/fqtemplate$ ls
build  CHANGELOG.rst  config  CONTRIBUTING  doc  fqtemplate  include  install  launch  LICENSE  log  README  src  test
```

What I don't see are any **msg**-type folders. Looking at [an example](https://github.com/ros2/common_interfaces/tree/master/std_msgs "std_msgs") I see just a msg folder. It is probably best practice to have message, service, and action definitions in a separate package. Can I find a Best Practices reference for that?

Also note that /include usually includes the packages name, here that should be /include/fqtemplate/example.hpp

For documentation, see [rosdoc2](https://github.com/ros-infrastructure/rosdoc2)

I see some **.circleci** subdirectory [here](https://github.com/ros-planning/navigation2) I need to investigate intergrating CI into my project outside of the ROS build farm. There is also a Github CI available. There is also a [ROS CI package](https://github.com/ros2/ci) based on Jenkins. Should I try to setup that?

OK I messed up and mixed workspace and package config. Try again. (Actually start over). Let's use templates as the workspace, and fqdemo as the project./ Convert the cpp demo code there.

Added a license file from [here](https://www.apache.org/licenses/LICENSE-2.0), README.md, CHANGELOG.rst

Now I have this structure:
```bash
kent@ubutower:~/ros2_wses/templates/src/fqdemo$ ls -R
.:
CHANGELOG.rst  CMakeLists.txt  config  CONTRIBUTING  doc  fqdemo  include  launch  LICENSE  package.xml  README.md  src  test

./config:
dummy.yaml

./doc:
dummy.md

./fqdemo:
dummy.py

./include:
fqdemo

./include/fqdemo:
dummy.hpp

./launch:
dummy.xml

./src:
publisher_member_function.cpp

./test:
```

How would this be stored? As a single repository. Eventually I'll call it fqdemo. Heck why not start today. I'll create this on github rosdabbler.

OK, it's [fqdemo](https://github.com/rosdabbler/fqdemo)

## Tests
So I would like to write a test. How to do that? I see lots of packages that are related to a test framework: ros2test, ament_cmake_test. But most have virtually no documentation. Some searching through https://answers.ros.org reveals `colcon test` which is the answer to "To run tests for the packages we just built, run the following:" but so far no simple, decent example. Am I going to have to spend a couple of months learning this before I can make any sense of it?

The [ros_testing](https://github.com/ros2/ros_testing) repo has zero documentation that I can find.

The Ament testing has [this section](https://github.com/ros2/ros_testing) which is a little more helpful. [System Dependencies](https://index.ros.org/search/?term=name%3A+%2Agtest%2A&section=deps) of gtest make lead me to some examples. Some more searching on https://index.ros.org ame up with this [list of packages that use gtest](https://index.ros.org/p/ament_cmake_gtest/github-ament-ament_cmake/#galactic-deps) The [sensor_msgs](https://github.com/ros2/common_interfaces/tree/galactic/sensor_msgs) package looks like it might be a good example. I might be able to duplicate that!

## Create a simple package.

First though I want to make a basic package that does something. Many of my planned nodes are transform nodes: they are given a stream of messages, and they transform that in some way to come up with an output. Like, given an audio message, come up with the Speech-To-Text output.

So I want to prepare a sample node that creates a message type with two integers, value and power, and returns two integers, exponent and root.

### Create a Custom Message type

See [Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html)

```bash
kent@ubutower:~/ros2_wses/templates/src$ ros2 pkg create --build-type ament_cmake fqdemo_msgs
```
then I roughly follow along the tutorial, but only msgs. Got the msgs package to build. Tried adding it to the fqdemo package, fails in find_package(fqdemo_msgs) ... OK got working.

## Next steps

With more searching, I decided that the ros-controls repos for ros2 are excellent guides for setup. I'm going to try to duplicate a lot of what they have.

See [ros-2-ci-action](https://github.com/marketplace/actions/ros-2-ci-action) for a ROS2-related Github action.