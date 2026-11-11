cmake_minimum_required(VERSION 3.16)
# Pick up the component that lives one folder up (this folder's parent).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cc1101_example)
