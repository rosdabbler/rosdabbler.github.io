---
title: "ROS2 package demo 17 - generated documentation"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-30
---
In fqdemo, there are a few documents I have included that should probaby be automatically generated. Experiement with locations for that.

I'll work with fqdemo_msgs snce it has the messages.

I'm having issues with "Contributing" appearing an excessive number of times in toctree glob. It appears to come from the myst include file:

    # CONTRIBUTING (myst)

    (This is an example of including external markdown, here to add a title. A common
    form of CONTRIBUTING.md does not include a title, and will not render).
    ```{include} ../../CONTRIBUTING.md
    ```
