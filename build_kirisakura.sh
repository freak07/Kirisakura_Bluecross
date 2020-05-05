#!/bin/bash

echo
echo "Clean Build Directory"
echo 

make clean && make mrproper

echo
echo "Issue Build Commands"
echo

mkdir -p out
export ARCH=arm64
export SUBARCH=arm64
export CLANG_PATH=~/Android_Build/Clang_Google/linux-x86/clang-r383902/bin
export PATH=${CLANG_PATH}:${PATH}
export CLANG_TRIPLE=aarch64-linux-gnu-
export CROSS_COMPILE=~/Android_Build/GCC_Google_Arm64/aarch64-linux-android-4.9/bin/aarch64-linux-android-
export CROSS_COMPILE_ARM32=~/Android_Build/GCC_Google_Arm32/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-
export LD_LIBRARY_PATH=~/Android_Build/Clang_Google/linux-x86/clang-r383902/lib64:$LD_LIBRARY_PATH

echo
echo "Set DEFCONFIG"
echo 
# make CC=clang O=out kirisakura_defconfig
make CC=clang O=out b1c1_defconfig

echo
echo "Build The Good Stuff"
echo 

make CC=clang O=out -j4
