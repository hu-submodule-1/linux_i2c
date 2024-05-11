cmake_minimum_required(VERSION 3.10)

# 定义静态库
add_library(linux_i2c STATIC i2c.c)

# 添加头文件搜索路径
target_include_directories(linux_i2c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
