#!/bin/bash

# SPDX-FileCopyrightText: Alliander N. V.
# 
# SPDX-License-Identifier: Apache-2.0
OUTPUT=$(doxygen doxyfile.lint /dev/null 2>&1)
NUM_WARNINGS=$(($(echo "$OUTPUT" | wc -l) - 1))

if [ "$NUM_WARNINGS" -gt 0 ]; then
  echo "$OUTPUT"
  echo "Doxygen found $NUM_WARNINGS documentation warnings."
  exit 1
fi
