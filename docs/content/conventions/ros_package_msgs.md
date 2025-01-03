<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# ros_package_msgs conventions

## Folder Structure

We define custom messages, services and actions in a separate package, with a package name ending on *_msgs*, to avoid build problems. This package has the following folder structure:

```text
ros_package/
|   CMakeLists.txt
|   package.xml
|
└───srv/
|   |   custom_service_definition.srv
|
└───msg/
└───action/
```

- The structure is very similar to a normal ROS package.
- There is a *srv*, *msg* and/or *action* directory for custom service/message/action definitions.

## CMakeLists.txt

Also the CMakeLists.txt file is very similar. This shows an example where only custom services are generated, but the procedure for custom messages or actions is very similar:

:::{literalinclude} example_files/ros_msgs_package/CMakeLists.txt
:language: CMake
:linenos:
:::

**10**:\
The *rosidl_default_generators* dependency is now required, to generated the custom messages.

**16-22**:\
We use CMake's *GLOB* method to automatically obtain all the srv files.

**24-27**:\
We generate the custom service and link the dependencies (if any).

## package.xml

To generate custom messages successfully, we also need to specify the following dependencies in the package.xml file:

```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
