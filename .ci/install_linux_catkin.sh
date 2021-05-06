#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"
cp -r "${GITHUB_WORKSPACE}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

$SUDO apt-get install -y clang-format-6.0

# Right now, build-test always builds adapy on focal
$SUDO apt-get -y install python3-dev python3-numpy
$SUDO apt-get -y install python3-pip -y
$SUDO pip3 install pytest -U
$SUDO apt-get -y install pybind11-dev python3 libpython3-dev python3-pytest \
  python3-distutils
