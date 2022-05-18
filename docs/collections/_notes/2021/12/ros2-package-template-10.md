---
title: "ROS2 package demo 10 - deploying rosdoc2 on github pages"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-16
---
Yesterday I made https://github.com/rosdabbler/fqdemo_ws public. There was an existing githb workflow from the template, that published a docs website on the gh-pages branch. To make that appear, in the repo settings/Pages I had to enable the Source branch, then the pages appeared. But I need for this to show the rosdoc2 result, using a github action.

I try to run rosdoc2 on fqdemo_nodes in its new location, it fails:
```bash
rosdoc2 build -p ./ -d doc/_build/build -c doc/_build/cross_reference -o doc/_build/output
...
Sphinx error:
root file /home/kent/ros2_wses/fqdemo_ws/src/fqdemo/fqdemo_nodes/doc/index.rst not found
```

Not sure what version of rosdoc2 or fqdemo_nodes this is, let me investigate.

I switched rosdoc2 to the prerun_conf_py branch and installed and ran, no help. Looking at my old fqdemo_nodes repo, there is an index.rst in the root that is not in the current version. What happened?

I took the old repo and copied it to the new location, ran rosdoc2 and it worked. Not sure how I messed up yesterday! Hmmm, "worked" as in ran to completion. But none of the code docs are showing, I need to fix all of this.

OK that's wierd, a separate bash console works, but the one in vscode does not. What gives? Restarted vscode, now it works.

As for missing stuff, the C++ api is now under "generated" instead of "api". Fixed in index.rst, now I see it.

After a bit more playing, I got it all working and landed a working demo on the gh-pages branch, see [the results](https://rosdabbler.github.io/fqdemo_ws/fqdemo_nodes/index.html)

## Metapackage

I want to use rosdoc2 to generate an overall documentation for this repo, including both packages possibly with cross-references. I made fqdemo_ws into a metapackage, following the example from https://github.com/ros-planning/navigation2. Now, how to add documentation to that?

I found a bug in my rosdoc2, pushed fix to my rosdoc2 prerun_conf_py branch.

Now I am starting to publish both fqdemo_nodes and fqdemo_ws results from rosdoc2. Time to figure out cross references, as I want to write an index.rst for fqdemo_ws that references its subpackage fqdemo_nodes.

As my first attempt at cross-references, I am running at the package.xml level:
```bash
rosdoc2 build -p ./ -d /tmp/doc/build -c /tmp/doc/cross_reference -o /tmp/doc/output --debug -u file:///tmp/doc/output
```
That gives me, in conf.py,
```python
    ensure_global('intersphinx_mapping', {
        'fqdemo_nodes': ('file:///tmp/doc/output/fqdemo_nodes/sphinx_html', '/tmp/doc/cross_reference/fqdemo_nodes/objects.inv')
    })
```
Hmmm, tried some things, did not work to link to fqdemo_nodes. Googled, can't find a single simple example. Frustrating. I guess I'll experiment.