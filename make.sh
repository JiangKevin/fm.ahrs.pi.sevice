clear
reset
# 
export FmDev=$(pwd)
export vcpkg=/home/kevin/Dev/tools/vcpkg
# 
rm -rf build
rm -rf bin/*.mat
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=/home/kevin/Dev/tools/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=install 
cmake --build build -j10
# # 
# cd bin
# # 
# ./ahrs_client