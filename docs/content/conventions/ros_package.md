<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# ros_package conventions

## Folder Structure

We try to follow this folder structure for the ROS packages:

```text
ros_package/
|   CMakeLists.txt
|   package.xml
|
└───src/
|   |   cpp_node.cpp
|
└───include/
|   └───ros_package/
|       |   cpp_header.hpp
|
└───src_py/
|   |   py_node.py
|
└───ros_package/
|   |   __init__.py
|   |   py_module.py
|
└───launch/
    |   launch_file.launch.py
```

- Every ROS package contains the `CMakeLists.txt` and `package.xml` in the root directory.
- In case of development using C++, the executables are located in the `src/` directory. The corresponding headers are located in the `include/` directory, inside a sub-directory that equals the package name.
- In case of development using Python, the executables are located in the `src_py/` directory.
- A sub-directory `ros_package/` with the same name as the ROS package can be used to create a Python package. This directory contains an `__init__.py` file and the Python modules of the Python package.
- Possible launch files are located in the `launch/` directory.
- More directories are possible, like `urdf/` for urdf files or `config/` for config files.

## CMakeLists.txt

This CMakeLists.txt file shows the different parts required to build a ROS package:

:::{literalinclude} example_files/ros_package/CMakeLists.txt
:language: CMake
:linenos:
:::

**5-10**:\
The file always starts with a version definition, the package name and the CMake dependencies when building C++ and/or python files.

**12-14**:\
If the package depends on other packages, these are defined. In this case, the packaged depends on the *vision_msgs* and *geometry_msgs* packages.

**16-22**:\
Building a C++ executable requires 3 steps: defining the executable, linking dependencies (if any) and installing the targets to the lib directory.

**24-28**:\
For Python executables, we can simply install them all at the same time, by  providing the directory.

**30-31**:\
If the package contains a Python package, it needs to be installed.

**33-37**:\
All shared folders are installed into the share directory. This includes the directory of launch files, but also other possible directories, like `urdf/` or `config/`, if these exist.

**39-45**:\
The file always ends with a default test and the *ament_package()* command.

## package.xml

The package.xml file is related to the CMakeLists.txt file:

:::{literalinclude} example_files/ros_package/package.xml
:language: xml
:linenos:
:::

**1-2**:\
The files starts with default xml definitions.

**10-15**:\
Inside the *package* tag, we start with some general information about the package.

**17-18**:\
Next, we define the build tool dependencies for building C++ and/or Python files.

**20-21**:\
Next, we define other packages where our package depends on.

**23-28**:\
The file ends with the default test dependency and an export definition.
