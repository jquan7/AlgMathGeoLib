#!/bin/bash

rm CMakeCache.txt
cmake -DMATH_TESTS_EXECUTABLE=1 -DCMAKE_BUILD_TYPE=Debug $* ..

echo "make --version"
make --version

echo " "

echo "gcc --version"
gcc --version
