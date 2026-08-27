#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Start vite dev in the background and set up signal traps to forward signals to the vite process:
bash -c "npm install && npm run dev" & pid=$!
trap 'kill -INT "$pid"' INT
trap 'kill -TERM "$pid"' TERM
wait "$pid"