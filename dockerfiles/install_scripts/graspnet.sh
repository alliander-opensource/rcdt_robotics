#!/bin/bash -i

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e
source /home/$UNAME/.bashrc
apt update

# Clone our graspnet fork:
cd /home/$UNAME
git clone https://github.com/alliander-opensource/graspnet-baseline.git
