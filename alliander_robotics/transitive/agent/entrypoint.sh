#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Set up colors for output:
NORMAL=$(tput sgr0)
UNDERLINE=$(tput smul)
RED=$(tput setaf 1)
BLUE=$(tput setaf 4)

# Print the header:
printf "${UNDERLINE}Transitive Agent.${NORMAL}\n\n"

# Check for required environment variables:
vars=(
    TR_INSTALL_HASH
    TR_USERID
    TR_ROBOT_TOKEN
    TR_DISPLAYNAME
    TR_LABELS
)
vars_unset=()
for var in "${vars[@]}"; do
    [ -z "${!var}" ] && vars_unset+=("$var")
    [ $var == "TR_ROBOT_TOKEN" ] && printf '%-18s %s\n' "$var:" "${!var:0:5}..."
    [ $var != "TR_ROBOT_TOKEN" ] && printf '%-18s %s\n' "$var:" "${!var}"
done
if [ ${#vars_unset[@]} -gt 0 ]; then
    printf "\n${RED}Error. The following variables are unset: ${vars_unset[*]}.${NORMAL}"
    printf "\n${RED}Please update ${HOME}/transitive.env and restart the container.${NORMAL}"
    exit 1
fi

# Copy the installation to the mounted directory if it doesn't exist yet:
if [ ! -e $HOME/.transitive/.installation_complete ]; then
    printf "\nTransitive installation not found in ${HOME}/.transitive. Copying files...\n"
    cp -r /transitive-preinstalled/. $HOME/.transitive
    rm -rf /transitive-preinstalled
fi;

# Set the environment variables in the .env and .token file:
cd $HOME/.transitive
sed -i "s/TR_USERID=.*/TR_USERID=$TR_USERID/" .env;
echo "TR_ROBOT_TOKEN=$TR_ROBOT_TOKEN" > .token;

# Start the agent in the background and set up signal traps to forward signals to the agent process:
printf "\n${BLUE}Starting Transitive agent...${NORMAL}\n\n"
bash start_agent.sh & pid=$!
trap 'kill -INT "$pid"' INT
trap 'kill -TERM "$pid"' TERM
wait "$pid"