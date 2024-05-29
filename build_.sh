pushd build
cmake --build . --config Release --target ORB_SLAM3 "$@"
popd
