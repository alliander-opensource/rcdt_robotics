#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
# 
# SPDX-License-Identifier: Apache-2.0

pydoclint *.py alliander_robotics --exclude=".venv/" -q
