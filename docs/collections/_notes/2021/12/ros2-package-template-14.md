---
title: "ROS2 package demo 14"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-20
---
In the previous notes, I noted the need to support sparse checkout with vcstool in order to have a viable way to use a single repo for both nodes and messages. Since then I made my own fork, and incorporated the patch. See https://github.com/rkent/vcstool. Tested it and it works, at least for a checkout.

## Github workflow

I started the course https://lab.github.com/githubtraining/github-actions:-hello-world to learn github actions.

2021-12-21: continuing with the course
2020-12-22: continuing with the course

To support the github branch/PR workflow, it helps to know the current branch. After some googling, I modified my bash prompt to support this, adding to ~/.bashrc:

```bash
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\xE2\x80\xA6\n(\1)/'
}
export PS1="\[\033[1;31m\]\u@\h:\[\033[32m\]\w\[\033[33m\]\$(parse_git_branch)\[\033[00m\] $ "
```
This adds an elipse, a new line, and the current git branch when on git repos.
