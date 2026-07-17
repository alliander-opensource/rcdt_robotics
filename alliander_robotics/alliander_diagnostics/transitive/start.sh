#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
# 
# SPDX-License-Identifier: Apache-2.0

if [ -z "$TR_INSTALL_HASH" ]; then
    echo "TR_INSTALL_HASH is not set. Please set this variable on your host machine to use transitive."
    exit 1
fi

echo "Starting transitive agent for device: '$TR_INSTALL_HASH'..."

# https://transitiverobotics.com/docs/guides/installing_in_docker/
if [ ! -e $HOME/.transitive/.installation_complete ]; then
  cp -r /transitive-preinstalled/. $HOME/.transitive
  rm -rf /transitive-preinstalled
fi;
cd $HOME/.transitive
bash start_agent.sh