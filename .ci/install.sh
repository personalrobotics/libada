#!/usr/bin/env bash

set -ex

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  if [ "${USE_CATKIN}" = "ON" ]; then
    . "${TRAVIS_BUILD_DIR}/.ci/install_linux_catkin.sh"
  fi
fi

