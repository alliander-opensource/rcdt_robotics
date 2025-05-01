#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

set -e

apt update
apt install -y libxcb-cursor-dev
pip install git+https://github.com/alliander-opensource/PyFlow.git@master
pip install \
    inflection \
    distinctipy
