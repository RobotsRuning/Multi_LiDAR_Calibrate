cmake_minimum_required(VERSION 3.27)
project(validation_algorithms)

set(CMAKE_CXX_STANDARD 17)

# 寻找PCL配置
find_package(PCL 1.8 REQUIRED)

# 添加PCL包含目录
include_directories(${PCL_INCLUDE_DIRS})

# 添加PCL定义和编译选项
add_definitions(${PCL_DEFINITIONS})

# 创建可执行文件
add_executable(validation_algorithms main.cpp)

# 链接PCL库
target_link_libraries(validation_algorithms ${PCL_LIBRARIES})
