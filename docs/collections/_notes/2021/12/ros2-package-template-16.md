---
title: "ROS2 package demo 16 - Repo as package documentation"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-28
---
I have reorganized the demo so that the repo is also a metapackage. This metapackage is also generating documentation at https://rosdabbler.github.io/fqdemo. Now I'm going to clean up some of the actual documentation, particularly of the repo itself which is new.

Issue: dot not found. I needed to install graphviz in the Dockerfile.

Issue: I'm getting these errors running rosdoc2, both locally and remotely:

```
WARNING: autodoc: failed to import module 'fqdemo_nodes'; the following exception was raised:
No module named 'fqdemo_nodes'
WARNING: autodoc: failed to import module 'PySubPub' from module 'fqdemo_nodes'; the following exception was raised:
No module named 'fqdemo_nodes'
```
Also, I have no documentation showing for the python modules. I need to investigate.

In the conf.py in the build directory, I have:
```python

    # Add the package directory to PATH, so that `sphinx-autodoc` can import it
    sys.path.insert(0, os.path.dirname('./fqdemo_nodes'))
```
That is not correct. Investigated and fixed, now it works.

2021-12-29

I'm trying to get various features working in the docs.

I created manually some documentation for the messages in fqdemo_msgs.

For C++, I am not seeing the members appear in for example Class DemoSubPub. I can make them appear using, in conf.py, :
```python
# breathe configuration
breathe_default_members = ('members', 'undoc-members')
```
But what I don't understand is that it is 'undoc-members' that is critical here. Why? They look documented to me!

OK the issue was not properly assigning to a tuple. This works:
```python
# breathe configuration
breathe_default_members = ('members', )
```

2021-12-30

To fix documentation issues, I want to try adding myst files to a rst wrapper. [Myst](https://myst-parser.readthedocs.io/en/latest/sphinx/use.html) shows how to do this, but it did not work for me.

Note says I need docutils>=0.17. I checked ```pip show docutils``` I had 0.16. Upgraded with ```sudo pip install --upgrade docutils``` and got a version >0.18, which then showed pip errors as conflicting with sphinx. So I did this and it seemed to be happy:
```bash
sudo pip install --upgrade docutils==0.17
```

I wanted to use rst because I had hoped I could add a title, but hide it. No luck. If contributing.md includes a title, it will get duplicated.

## Getting cross references from C++ to message files.

I have cross-references working from python to the message file with this syntax in a docstring:

```
    **Topics Subscribed**

    * ``num_power`` (type :py:class:`NumPwrData`)
        Requests from package users to calculate a power and root.

    **Topics Published**

    * ``power_result`` (type :py:class:`fqdemo_msgs:fqdemo_msgs.msg.NumPwrResult`)
        Message is published either in response to an incoming ``num_power``
        NumPwrResult message, or a message with zeros is published periodically.
```
That shows both an abbreviated as well as an extended format. But that does not work in C++.

[This reference](https://my-favorite-documentation-test.readthedocs.io/en/latest/using_intersphinx.html) gives hints how to make that work. Let me try that.

OK that works - but I found that the \rst block is sensitive to use of leading characters. Breathe documentation [describes this problem](https://breathe.readthedocs.io/en/latest/markups.html). So I need to have different \rst aliases depending on the style

I added this to fqdemo_nodes Doxyfile:
```
# See https://exhale.readthedocs.io/en/latest/mastering_doxygen.html#doxygen-aliases
# and https://my-favorite-documentation-test.readthedocs.io/en/latest/using_intersphinx.html

# My own edits based on https://breathe.readthedocs.io/en/latest/markups.html

ALIASES  = "endrst=\endverbatim"
# rst inline
ALIASES += "rst=\verbatim embed:rst:inline"
# rst block with leading blanks
ALIASES += "rstb=\verbatim embed:rst"
# rst block with leading asterisk
ALIASES += "rsta=\verbatim embed:rst:leading-asterisk"
# rst block with leading triple slashes
ALIASES += "rsts=\verbatim embed:rst:leading-slashes"
```

Now this doc block works, showing two different styles:
```
/** 

  A demonstration of a simple ROS2 node that raises numbers to a power and root

  @rstb
  **Topics Subscribed**: ``num_power`` (type :py:class:`NumPwrData`): Publishes a message to `/power_result`
  after message is received.
  @endrst

  <b>Topics Published:</b> `power_result` (type @rst :py:class:`NumPwrResult` @endrst ). A zero-valued message
  is published periodically. A message with appropriate values is published in response to
  a /num_power message.

*/
```
Note the blank after @endrst. Without the blank, this fails in Doxygen. Actually, with a default_domain of **any** then this also works:
```
(type @rst `NumPwrResult` @endrst )
```
