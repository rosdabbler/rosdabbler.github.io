---
title: "ROS2 package demo 8 - ros buildfarm scripts"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-01
---
I've been off my game writing notes the last few days. Let's get back to it!

# ROS buildfarm and its scripts

[Here](https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/doc_jobs.rst) there is documentation for creating various docs-related jobs on the buildfarm. The intent is to make it easy for people to apply this locally. Let me try to do that with **rosdoc2**. AFACT the scripts support this, but I have not found any evidence of their actually being used.

The [ROS2 Roadmap](https://docs.ros.org/en/ros2_documentation/galactic/Roadmap.html) claims for **Humble** due in May 2022:

* - Documentation: Add Python API support to rosdoc2
  - Medium
  - Open Robotics
  - 4th quarter 2021
* - Documentation: Support inter-package linking for rosdoc2 buildfarm jobs
  - Small
  - Open Robotics
  - 1st quarter 2022
* - Documentation: Upload C++ API documentation for core packages to docs site
  - Medium
  - Open Robotics
  - 4th quarter 2021

Maybe I could help out? But in any case, my template should/could use standard scripts for generating docs, that would help make sure things are generated according to standards. My goal today is to figure out what those scripts are, and how to use them.

I see in *ros_buildfarm/templates/doc/rosdoc2_task.Dockerfile.em* that there is a rosdoc2 type in the jobs that *is not* documented in the .rst file. I also see in `21,21: DOC_TYPE_ROSDOC2 = 'rosdoc2'` in */home/kent/projects/jenkins/private_repos/ros_buildfarm/ros_buildfarm/config/doc_build_file.py* alongside `DOC_TYPE_ROSDOC = 'rosdoc_lite'` which *is* documented.

Maybe what I should do is try to run a rosdoc_lite job first, which AFAICT is being actually used to generate stuff, then adapt it to run rosdoc2. That is, from the docs, :

* related to the **rosdoc_lite** type:
  * **generate_doc_script.py** generates a *shell* script which will run the
    same tasks as the *doc* job for a specific doc repository on a local
    machine.

So that is my goal, **generate_doc_script.py** for a rosdoc_lite. Let me try that for a message-only repo.

Oh, great. Installing chef-workstation yesterday broke my github pages jekyll setup. Oh, good, removing it fixed that.

[This] suggests I use a venv to do this stuff. I generally manage without, let me do it that way.

```bash
sudo su
pip3 install empy
pip3 install jenkinsapi
pip3 install rosdistro
pip3 install ros_buildfarm
```

All installed. [Docs](https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/doc_jobs.rst#example-invocation) gives the example:

```bash
mkdir /tmp/doc_job
generate_doc_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml indigo default roscpp_core ubuntu trusty amd64 > /tmp/doc_job/doc_job_indigo_roscpp_core.sh
cd /tmp/doc_job
sh doc_job_indigo_roscpp_core.sh
```
Cool, this is available locally from the install:
```bash
kent@ubutower:~/projects/package-template/scripts$ generate_doc_script.py
usage: generate_doc_script.py [-h] [--force]
                              config_url rosdistro_name doc_build_name repository_name os_name os_code_name
                              arch
generate_doc_script.py: error: the following arguments are required: config_url, rosdistro_name, doc_build_name, repository_name, os_name, os_code_name, arch
kent@ubutower:~/projects/package-template/scripts$ which generate_doc_script.py
/usr/local/bin/generate_doc_script.py
```

Let me try to get docs for std_msgs. [Here](http://docs.ros.org/en/noetic/api/std_msgs/html/index-msg.html) is the doxygen result. I want to try to create that. No, let me use a less systemy type, how about [radar_msgs](http://docs.ros.org/en/noetic/api/radar_msgs/html/index-msg.html)?

```bash
generate_doc_script.py https://raw.githubusercontent.com/ros-infrastructure/ros_buildfarm_config/production/index.yaml noetic default radar_msgs ubuntu focal amd64 > test.sh
```
Now I am trying to understand that file. I'm nervous abut the notifications. I have my own ros_buildfarm_config that has purged those, let me try that.
```bash
generate_doc_script.py file:/home/kent/projects/package-template/private_repos/ros_buildfarm_config/index.yaml noetic default radar_msgs ubuntu focal amd64 > radar_msgs_noetic.sh
```

OK I can run that, but it dies because it cannot fine the file with the config. It dies in *Build step 8; # BEGIN SECTION: Run Dockerfile - generating doc task*. I uess they really want that URL to be accessible using http, which is a pain because it forces me to get the docker file to be able to read my private repo.

I think what I will do temporarily is just move that to a gist, since it does not actually have anything private yet.

Gist url is [this](https://gist.githubusercontent.com/rkent/70a29d6316fda664b6377dfa639815f1/raw/71c9dd4d448d02daf877a2744b0be5be79055c0d/index.yaml)

```bash
generate_doc_script.py  https://gist.githubusercontent.com/rkent/70a29d6316fda664b6377dfa639815f1/raw/71c9dd4d448d02daf877a2744b0be5be79055c0d/index.yaml noetic default radar_msgs ubuntu focal amd64 > radar_msgs_noetic.sh
```

That did not work, it needs the full repository. I give up, I'm going to make it public for now in spite of their warnings.

```bash
generate_doc_script.py https://raw.githubusercontent.com/rkent/ros_buildfarm_config/production/index.yaml noetic default radar_msgs ubuntu focal amd64 > radar_msgs_noetic.sh
```

OK that ran, got something interesting here: file:///home/kent/projects/package-template/ws/generated_documentation/api_rosdoc/radar_msgs/html/msg/RadarReturn.html

The next question is where do I put the type of document run to do? Did that get expanded from the "default"?

This is the name under doc_builds in the index.yaml:
```yaml
  noetic:
   doc_builds:
      default: noetic/doc-build.yaml
     released-packages-without-doc-job: noetic/doc-released-build.yaml
```

The file doc-build.yaml starts with:
```yaml
%YAML 1.1
# ROS buildfarm doc-build file
---
build_environment_variables:
  ROS_PYTHON_VERSION: 3
canonical_base_url: http://docs.ros.org/en
documentation_type: rosdoc_lite
```
so that is where the 'rosdoc_lite' comes from.

How does this relate to ros2? I found the [buildfarm configs](https://github.com/ros2/ros_buildfarm_config)

Now I have found where rosdoc2 is being run on rolling packages [here!](http://docs.ros.org/en/ros2_packages/rolling/api/)

So what would it take to run the standard script on my package? Let me first run this script using the existing tooling on an existing package.

I added `rolling` directory from ros2 ros_buildfarm_config to my repo. Now let me try to create a job, repo ros-perception/image_common:
```bash
generate_doc_script.py https://raw.githubusercontent.com/rkent/ros_buildfarm_config/production/index.yaml rolling default ros-perception/image_common ubuntu focal amd64 > image_common_rolling.sh
```

Oops, distribution.yaml for rolling is [here](https://github.com/ros/rosdistro/blob/master/rolling/distribution.yaml). Repo is image_common. Try again:
```bash
generate_doc_script.py https://raw.githubusercontent.com/rkent/ros_buildfarm_config/production/index.yaml rolling default image_common ubuntu focal amd64 > image_common_rolling.sh
```

Script created, try running. Ran to the end, with a few errors:
```
[rosdoc2] [INFO] Running Sphinx-build: 'sphinx-build -c . default_sphinx_project /tmp/ws/docs_build/camera_info_manager/camera_info_manager/sphinx_output' in 'docs_build/camera_info_manager/camera_info_manager'
Running Sphinx v4.3.1

Configuration error:
There is a syntax error in your configuration file: invalid syntax (<string>, line 26)

/tmp/ws/docs_build/camera_info_manager/camera_info_manager/conf.py
```
but we got as far as image_transport, and this indeed looks like I expect.