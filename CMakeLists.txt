cmake_minimum_required(VERSION 3.15)

set(app_name "ahrs_sevice")

# Set project name
project(${app_name})

# 设置所有可执行文件的运行时输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{FmDev}/bin)
# C++17 is required for this project.
set(CMAKE_CXX_STANDARD 20)

# 添加编译选项
add_compile_options(
    -Wnarrowing 
)

#
include_directories($ENV{FmDev}/source)
include_directories($ENV{FmDev}/source/device)
include_directories($ENV{FmDev}/source/device/TDK40607P)
include_directories($ENV{FmDev}/libs/libgpiod/include)
include_directories($ENV{FmDev}/libs/openkf/include)


#
# add_definitions(-DENABLE_LOGGING)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")
find_package(nlohmann_json CONFIG REQUIRED)
find_package(unofficial-concurrentqueue CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(SQLiteCpp CONFIG REQUIRED)
find_package (GeographicLib CONFIG REQUIRED)
#
# 查找源文件
file(GLOB app_src
    $ENV{FmDev}/source/*.cpp
    $ENV{FmDev}/source/*.inl
    $ENV{FmDev}/source/*/*.cpp
    $ENV{FmDev}/source/*/*.inl
    $ENV{FmDev}/source/*/*.c
    $ENV{FmDev}/source/*/*/*.cpp
    $ENV{FmDev}/source/*/*/*.inl
    $ENV{FmDev}/source/*/*/*.c
    $ENV{FmDev}/source/*/*/*/*.cpp
    $ENV{FmDev}/source/*/*/*/*.inl
    $ENV{FmDev}/source/*/*/*/*.c
)

# 添加可执行文件
add_executable(${app_name}
    ${app_src}
)

# 链接库到可执行文件
target_link_libraries(${app_name} PUBLIC
    unofficial::concurrentqueue::concurrentqueue
    nlohmann_json::nlohmann_json
    #
    $ENV{FmDev}/libs/libgpiod/lib/libgpiod.a
    $ENV{FmDev}/libs/openkf/lib/libOpenKF.a
    Eigen3::Eigen
    SQLiteCpp
    ${GeographicLib_LIBRARIES}
)