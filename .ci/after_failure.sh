#!/usr/bin/env bash

set -ex

if [ "${USE_CATKIN}" = "ON" ]; then
  cd "${HOME}/workspace/build/libada/"
fi

cat Testing/Temporary/LastTest.log
cat Testing/Temporary/LastTestsFailed.log

