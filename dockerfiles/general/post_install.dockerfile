# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Define GZ_SIM_RESOURCE_PATH:
ENV GZ_SIM_RESOURCE_PATH="/home/$UNAME/franka_ws/src/"

# Define IGN_IP to use gazebo with firewall enabled:
ENV IGN_IP=127.0.0.1

#Set user and workdir:
RUN chown -R $UNAME /home/$UNAME
USER $UNAME
WORKDIR /home/$UNAME

#Set entrypoint to bash:
ENTRYPOINT ["/bin/bash"]