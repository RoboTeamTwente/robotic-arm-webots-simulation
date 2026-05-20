cmake_minimum_required(VERSION 3.20)

project(robotic_arm_sim VERSION 0.1)

add_executable(my_arm_controller my_arm_controller.c)
target_compile_options(my_arm_controller PRIVATE -Wall -Wextra)

set(CMAKE_BUILD_TYPE Debug)

#including control and webots include paths
target_include_directories(my_arm_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/erc-control-arm/control_arm_webots_ert_rtw $ENV{WEBOTS_HOME}/include/controller/c)

#adding control
add_subdirectory(erc-control-arm)

#linking
target_link_libraries(my_arm_controller webots_automatic $ENV{WEBOTS_HOME}/lib/controller/libController.so m)
