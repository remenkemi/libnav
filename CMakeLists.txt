cmake_minimum_required(VERSION 3.13)
project(nav-test CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -fPIC -g")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src/lib)

add_executable(navcmd "src/main.cpp")
target_link_libraries(navcmd PUBLIC libnav)

set(PLUGIN_BUILD_DIR "${PROJECT_SOURCE_DIR}")

set_target_properties(navcmd PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PLUGIN_BUILD_DIR}")
