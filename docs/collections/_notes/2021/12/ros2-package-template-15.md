---
title: "ROS2 package demo 15 - rosdoc2 container and github actions"
layout: rkjnote
author_profile: true
tags: ['package template']
date: 2021-12-23
---
Let's setup a container to run rosdoc2, publishing to github.

What am I trying to accomplish? I want to be able to run rosdoc2 on a package folder under Docker, in a way that I can both run manually, as well as in a github workflow.

2021-12-27

I have a container demo in my ~/projects/rosdoc2 folder. I also reorganized the repo **fqdemo** so that it serves as the metaproject, with **fqdemo_msgs** and **fqdemo_nodes** underneath it. I can run rosdoc2 manually on each package. When I do, then I have an issue with the intersphinx links. There is a single base_url for the links. That means I will not be able to point my intershpinx directory to a public source, while also linking locally.

Or can I? Maybe the intersphinx directory is for the public, and I need to manually include the local links. Let me play with that.

In rosdoc2 sphinx_builder.py, the links are read from the cross_reference_directory:
```python
        # Collect intersphinx mapping extensions from discovered inventory files.
        inventory_files = \
            collect_inventory_files(self.build_context.tool_options.cross_reference_directory)
        base_url = self.build_context.tool_options.base_url
        intersphinx_mapping_extensions = [
            f"'{package_name}': "
            f"('{base_url}/{package_name}/{inventory_dict['location_data']['relative_root']}', "
            f"'{os.path.abspath(inventory_dict['inventory_file'])}')"
            for package_name, inventory_dict in inventory_files.items()
            # Exclude ourselves.
            if package_name != self.build_context.package.name
        ]
```
Results from the current run are copied into that same directory:
```python
        # Copy the inventory file into the cross-reference directory, but also leave the output.
        inventory_file_name = os.path.join(sphinx_output_dir, 'objects.inv')
        destination = os.path.join(
            self.build_context.tool_options.cross_reference_directory,
            self.build_context.package.name,
            os.path.basename(inventory_file_name))
        logger.info(
            f"Moving inventory file '{inventory_file_name}' into "
            f"cross-reference directory '{destination}'")
        os.makedirs(os.path.dirname(destination), exist_ok=True)
        shutil.copy(
            os.path.abspath(inventory_file_name),
            os.path.abspath(destination)
        )
```

For the existing ros2 documentation, I can find the cross-reference files at, for example, [This link](docs.ros.org/en/ros2_packages/rolling/api/action_tutorials_cpp/objects.inv). If I follow the logic of the existing **rosdoc2**, then what is missing is the ability to link to an external source of references, ie that directory.

For now, let me get my demo working on github with just local cross references.

2021-12-28

I did a manual run of rosdoc2 in fqdemo and its packages, manually copied the results to a **docs** directory and branch, and pushed to github. I learned that you need a **.nojekyll** file in the root in order for directories beginning with underscore to appear, as is needed in sphinx.

So now I have https://rosdabbler.github.io/fqdemo/ worjking - but I need to adjust the base_url to see the links to the packages.

Roughly following ros conventions, the html directory structure would be something like this:

```
docs
  fqdemo
    index.html
  fqdemo_nodes
    index.html
  fqdemo_msgs
    index.html
```
But github pages requires that I have:
```
docs
  index.html
```
So I need to have html to forward the opening page to the fqdemo/index.html file. Got it working, pushed, and it works. Now to fix the base_url.

Oops, base_url is a command line option, not a yaml. Got it working running manually.

After many tweaks and commits, I finally have github actions automatically running and generating docs! Time to move on.
