---
title: "ROS2 package demo 13 - Repo format with sparse checkout"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-19
---

I'm trying to figure out how to have a workspace like this:
```
package.xml
src/
  some_repo/
    package1_of_n/
  another_repo
    package3_of_n/
    package5_of_n/

Why? I need to be able to include the msg packages without the nodes.

Apparently this can be done with sparse checkout. This has been supported since 2012 in git, so this should work OK. There is another git feature using git clone --filter that is a recent development and allows you to do a partial clone as well. Not really needed by us though, as there are not large monorepos to deal with.

Initializing sparse checkout is a but tricky. You have to create the directory first, set the sparse feature and directories, then complete a pull. See [How do I clone a subdirectory only of a Git repository?](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository) for some details.

vcstool needs to support this. There is an existinfg [PR](https://github.com/dirk-thomas/vcstool/pull/219) to support this feature. Using this, it makes sense to group together related packages in a repo, including both the msg package and the node package. When using the repo, if the nodes are run remotely, the robot workspace need only checkout the msg folder.

I need to demo this.
