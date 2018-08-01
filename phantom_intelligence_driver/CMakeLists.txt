cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(phantom_intelligence_driver)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-module/")
include(cmake-module/phantom_intelligence_gateway.cmake)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        phantom_intelligence_msgs
        )

fetch_phantom_intelligence_gateway(
        ${PROJECT_SOURCE_DIR}/cmake-module
        ${PROJECT_BINARY_DIR}/lib/phantom_intelligence_gateway
)

find_package(catkin REQUIRED COMPONENTS ${{PROJECT_NAME}_CATKIN_DEPS})

