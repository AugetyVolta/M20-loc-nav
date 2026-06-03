#!/bin/bash
set -e

ROOT_DIR=$(cd $(dirname "$0"); pwd)

# build gtsam
cd ${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1
if [ ! -f install/lib/libgtsam.so ] && [ ! -f install/lib/libgtsam.a ]; then
  rm -rf build install
  mkdir build install
  cd build
  cmake .. \
    -DCMAKE_INSTALL_PREFIX="../install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  make -j6 && make install
fi

# build osqp
cd ${ROOT_DIR}/lib/3rdparty/osqp
if [ ! -f install/lib/libosqp.so ] && [ ! -f install/lib/libosqp.a ]; then
  rm -rf build install
  mkdir build && cd build
  cmake .. \
    -DCMAKE_INSTALL_PREFIX="../install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DOSQP_BUILD_TESTS=OFF \
    -DOSQP_BUILD_UNITTESTS=OFF \
    -DOSQP_BUILD_EXAMPLES=OFF \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  make -j4 && make install
fi
