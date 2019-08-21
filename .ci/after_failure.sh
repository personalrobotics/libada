#!/usr/bin/env bash

set -ex

cd "${HOME}/workspace"

if [ "${TRAVIS_OS_NAME}" = "linux" ]; then
  cat ./build/libada/Testing/Temporary/LastTest.log
  cat ./build/libada/Testing/Temporary/LastTestsFailed.log
fi
