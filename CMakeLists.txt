cmake_minimum_required (VERSION 3.22)
project(BME2104)

set(CMAKE_CXX_STANDARD 17) # C++17
set(CMAKE_C_STANDARD 11) # C11

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O2)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB SOURCES "*.h" "*.cpp")
list(REMOVE_ITEM SOURCES "${PROJECT_SOURCE_DIR}/main.cpp")
add_library(CodeCraft ${SOURCES})

add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(main CodeCraft)
set_target_properties(main PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# 测试程序
# add_executable(test-port_dis_init ${PROJECT_SOURCE_DIR}/test/test-port_dis_init.cpp)
# target_link_libraries(test-port_dis_init CodeCraft)
# set_target_properties(test-port_dis_init PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-astar ${PROJECT_SOURCE_DIR}/test/test-astar.cpp)
# target_link_libraries(test-astar CodeCraft)
# set_target_properties(test-astar PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-one_robot ${PROJECT_SOURCE_DIR}/test/test-one_robot.cpp)
# target_link_libraries(test-one_robot CodeCraft)
# set_target_properties(test-one_robot PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-just_move ${PROJECT_SOURCE_DIR}/test/test-just_move.cpp)
# target_link_libraries(test-just_move CodeCraft)
# set_target_properties(test-just_move PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(20240309score ${PROJECT_SOURCE_DIR}/release/20240309score.cpp)
# target_link_libraries(20240309score CodeCraft)
# set_target_properties(20240309score PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-astarwithcbs ${PROJECT_SOURCE_DIR}/test/test-astarwithcbs.cpp)
# set_target_properties(test-astarwithcbs PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-two_robot ${PROJECT_SOURCE_DIR}/test/test-two_robot.cpp)
# target_link_libraries(test-two_robot CodeCraft)
# set_target_properties(test-two_robot PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-ten_robot ${PROJECT_SOURCE_DIR}/test/test-ten_robot.cpp)
# target_link_libraries(test-ten_robot CodeCraft)
# set_target_properties(test-ten_robot PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-move_check ${PROJECT_SOURCE_DIR}/test/test-move_check.cpp)
# target_link_libraries(test-move_check CodeCraft)
# set_target_properties(test-move_check PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# add_executable(test-astarVex ${PROJECT_SOURCE_DIR}/test/test-astarVex.cpp)
# set_target_properties(test-astarVex PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)