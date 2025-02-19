<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Docker

Docker enables easy creation of complete system images. This simplifies the installation process of all the software required in a (robotic) project and ensures consistency between different users/devices.

## Installation

Follow [these](https://docs.docker.com/engine/install/ubuntu/) instructions to install Docker Engine in your Ubuntu system. Next, [add yourself to the docker group](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to avoid the use of sudo when running docker commands. The installation is successful when the *hello-world* example is working without sudo.

### Nvidia

If you have an Nvidia GPU, follow the [Installing with Apt](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt) instructions to install the NVIDIA Container Toolkit. Next, configure docker by following [these](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker) two steps. Finally, you can test whether the installation was successful by [running](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html#running-a-sample-workload-with-docker) the `nvidia-smi` command in docker:

### Visual Studio Code

It is recommend to use vscode for code development in combination with the docker images. It is possible to "attach" to a running docker container in vscode using the [Dev Containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). With this plugin installed and a docker container running, it should be possible to attach to this container by click on the *Open a Remote Window* button (blue button at the bottom left corner in vscode). This should give the option to *Attach to running container...*.

A new vscode window should be opened, connected to the docker container. This means that vscode should be able to find all (ROS) packages installed in the container. It is also recommend to install the [Ruff plugin](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff). This plugin enables linting (code style and quality checks) and formatting. The linting rules are automatically installed in the docker images as a file called `pyproject.toml` in the home directory. Ruff should be able to detect this file and follow the rules without further configuration.

## Image

A Docker image is build based on a Dockerfile. We use the file `dockerfiles/rcdt_robotics.Dockerfile`. This file contains the steps that Docker should execute to create our Docker image. Almost all steps are commands to install the (ROS) software, required in our projects.

When changes are made and a pull request is made, Github will automatically build the new image and push it to our [Docker hub](https://hub.docker.com/r/rcdt/robotics). You can automatically start the newest image with the *run* script in the root of our repository.

When you run a Docker image, note that all changes (installing software, creating files, defining environment variables) will be lost when closing it. The only exception are changes to files in the *rcdt_robotics* directory.

If you want to add software to the Docker image permanently, one should change the Dockerfile. One can create a install script for the software in `dockerfiles/install_scripts`. Make sure to start the script with the `set -e` command, so that Docker will stop building when errors occur. The script can be added in the `dockerfiles/rcdt_robotics.Dockerfile`. Scripts that take a long time to execute and don't change often are placed high in the file, since Docker creates cache after each *RUN* command.
