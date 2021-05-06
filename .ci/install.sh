#!/usr/bin/env bash

set -ex
if [ "${USE_CATKIN}" = "ON" ]; then
  . "${GITHUB_WORKSPACE}/.ci/install_linux_catkin.sh"
fi
