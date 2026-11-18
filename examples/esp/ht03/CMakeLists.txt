cmake_minimum_required(VERSION 3.16)

# Pull in the component straight from the repo root, and mcu-can next to it.
# This repo's folder may have any name: main needs no REQUIRES, as IDF gives
# the main component every component in the build. mcu-can's may not: IDF
# names a local component after its folder, and this component REQUIRES
# can, so mcu-can must be checked out next to this repo as can.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../.."
                         "${CMAKE_CURRENT_LIST_DIR}/../../../../can")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esc_ht03)
