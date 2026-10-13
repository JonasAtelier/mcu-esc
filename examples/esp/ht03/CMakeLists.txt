cmake_minimum_required(VERSION 3.16)

# Pull in the component straight from the repo root, and mcu-can next to it.
# A local component is named after its directory, so this assumes the repos
# are checked out as esc and can -- the names esp-forge gives them, and
# what main/CMakeLists.txt REQUIRES.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../.."
                         "${CMAKE_CURRENT_LIST_DIR}/../../../../can")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esc_ht03)
