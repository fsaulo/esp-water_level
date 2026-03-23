# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(DEFINED ENV{RMAKER_PATH})
    set(RMAKER_PATH $ENV{RMAKER_PATH})
endif(DEFINED ENV{RMAKER_PATH})

set(PROJECT_VER "1.8")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(water_level)
