cmake_minimum_required (VERSION 3.8)
project(CodeCraftSDK)

set(CMAKE_CXX_STANDARD 17) # C++17
set(CMAKE_C_STANDARD 11) # C11

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include) # 添加头文件路径
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src src_dir) # 添加源文件路径
add_library(CodeCraftSDK SHARED ${src_dir}) # 生成动态库
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}) # 设置可执行文件输出路径

# 生成用于提交的可执行文件
add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp) # 生成可执行文件
target_link_libraries(main CodeCraftSDK) # 链接动态库

# 生成测试用的可执行文件
add_executable(Test1 ${PROJECT_SOURCE_DIR}/tests/test.cpp) # 生成可执行文件
target_link_libraries(Test1 CodeCraftSDK) # 链接动态库
