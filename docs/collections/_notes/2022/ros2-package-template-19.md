---
title: "ROS2 package demo 19 - template for index.rst"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2022-01-02
---

I want to use a template for index.rst.

Issues:
* I would like to keep things under generated if possible

But I am finding that is not possible for index.rst. I tried naming the templated result index_generated.rst, and renaming the html file to index.html, but this results in broken home links.

So instead, the convention is if you have a template file, call it .j2 and it will overright the equivalent file.

2022-01-03

I'm still struggling with the issue of when to create conf.py, and when to update.

The readme says:

> In many cases, C/C++ packages require no configuration, and will work if you simply layout your package in a standard configuration and the tool will do the rest.

> However, if you want to provide additional documentation, like a conceptual overview or tutorials, then you will want to provide a Sphinx `conf.py` file and do that documentation in `.rst` files using Sphinx.

Looking again at the source, I see I was wrong about one thing. Default index.rst and conf.rst are only created in the generated build directory, not in the user's `doc` folder. What does that mean for use of templates?

For now, I'll just leave it so that conf.py and index.rst get overwritten if the .j2 versions exist.

2022-01-04

I've added now tags and template_variables to support linking to URLs found in package.xml.

2022-01-06

Made more changes. But at this point, the key issue is whether they want to go that direction or not. I added an issue on the main site, for now I'm going to do some other things.

