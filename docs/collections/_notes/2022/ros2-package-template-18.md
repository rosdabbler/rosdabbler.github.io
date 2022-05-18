---
title: "ROS2 package demo 18 - Rebase of rosdoc2 variations"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2022-01-01
---
I've made quite a mess of all of the variations of rosdoc2. I had hoped that aprotyas would do a rebase of his stuff, but I really need to move on regardless. So let me try to reorganize things to a more coherent structure.

These types of git mods are new to me, so I am learning as I go. I'm going to do these in a copied rkent/rosdoc2 directory so I can start over again easily.

## Rebase and fix of aprotyas/build-python-pkg

```bash
git checkout aprotyas/build-python-pkg
git rev-parse --short HEAD
# eeaed0d
# I'm going to squash build-python-pkg to a single commit in a new branch prior to rebase
git checkout -b rkent/squashed-build-python-pkg
# Find the common base
git merge-base rkent/squashed-build-python-pkg main
# fa887cbfd1c579209b34f358b78d765404bbd108
git reset --soft fa887cbfd1c579209b34f358b78d765404bbd108
git commit -m "squashed PR #29 - Add support for python-only projects (commit eeaed0d)"
git rebase main
# fix conflicts
git add .
git rebase --continue
```
That does the rebase. Now I am going to apply my fixes to that PR.
```bash
git checkout -b rkent/continue-build-python-pkg
git cherry-pick 81c2a6c
git cherry-pick 4c7c891
git cherry-pick eed8e52
git cherry-pick 6fe3285
```

## Rebase of additional changes
So at this point, rkent/continue-build-python-pkg has rebased changes that could work in a PR, with the same intent as the original PR but with a few fixes. To get to my currently runnign code, I need a few more changes.

Running conf.py in its original location:
```bash
git checkout -b rkent/prerun-conf
git cherry-pick 008f8cb
```
Using templates with conf.py:
```bash
git checkout -b rkent/jinja-templates
git cherry-pick 71e1ec7
```

I tried this out with:
```bash
rosdoc2 build -d /tmp/rd/2/build -c /tmp/rd/2/cr -o /tmp/rd/2/output --debug -u /tmp/rd/2/output -p ./fqdemo_nodes
rosdoc2 build -d /tmp/rd/2/build -c /tmp/rd/2/cr -o /tmp/rd/2/output --debug -u /tmp/rd/2/output -p ./fqdemo_sgs
rosdoc2 build -d /tmp/rd/2/build -c /tmp/rd/2/cr -o /tmp/rd/2/output --debug -u /tmp/rd/2/output -p ./
```
and it seems to work.
