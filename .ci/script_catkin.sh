#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

export PACKAGE_NAMES="$(./scripts/internal-get-packages.py distribution.yml ${REPOSITORY})"
./scripts/internal-build.sh ${PACKAGE_NAMES}

if [ $BUILD_NAME = DOCS ]; then
  . "${GITHUB_WORKSPACE}/.ci/build_docs.sh"
  exit 0
fi

# Check code style
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --make-args check-format -- libada

# Manually build libada's tests; they are not built automatically because it is not a Catkin package.
./scripts/internal-run.sh catkin build --no-status --no-deps -p 1 -i --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DTREAT_WARNINGS_AS_ERRORS=ON -DCODECOV=OFF --make-args tests -- libada

# Run tests and measure test coverage if CodeCov is on.
./scripts/internal-run.sh env CTEST_OUTPUT_ON_FAILURE=true make -C build/libada test
