cmake_minimum_required(VERSION 3.27)
project(lidar_calibrate)

set(CMAKE_CXX_STANDARD 17)
# 找到PCL库，确保包含了你需要的所有组件
find_package(PCL 1.8 REQUIRED COMPONENTS common io segmentation filters visualization)
# 找到Ceres Solver库
find_package(Ceres REQUIRED)

# 包含PCL和项目的头文件目录
include_directories(${PCL_INCLUDE_DIRS} include)
# 包含Ceres Solver的头文件目录
include_directories(${CERES_INCLUDE_DIRS})

# 添加PCL库的定义和链接目录
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# 找到Boost库，如果你的代码中没有使用到Boost，这部分可以省略
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 指定可执行文件和相关的源文件
add_executable(lidar_calibrate main.cpp src/feature_extraction.cpp  src/registration.cpp)

# 链接PCL库和Boost库（如果使用）到你的可执行文件
target_link_libraries(lidar_calibrate ${PCL_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${CERES_LIBRARIES})



