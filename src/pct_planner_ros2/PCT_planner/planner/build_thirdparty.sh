ROOT_DIR=$(cd $(dirname "$0"); pwd)

# build gtsam
cd ${ROOT_DIR}/lib/3rdparty/gtsam-4.1.1
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
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --target install -j4

# build osqp
cd ${ROOT_DIR}/lib/3rdparty/osqp
rm -rf build install
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build . --target install -j4
