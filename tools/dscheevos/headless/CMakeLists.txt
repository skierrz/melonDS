cmake_minimum_required(VERSION 3.16)
project(melonds_headless CXX C)
set(CMAKE_CXX_STANDARD 20)

set(BUILD_QT_SDL OFF CACHE BOOL "" FORCE)
set(ENABLE_JIT OFF CACHE BOOL "" FORCE)
set(ENABLE_OGLRENDERER OFF CACHE BOOL "" FORCE)
set(ENABLE_GDBSTUB ON CACHE BOOL "" FORCE)
set(MELONDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../.. CACHE PATH "melonDS source root")
add_subdirectory(${MELONDS_DIR} melonds EXCLUDE_FROM_ALL)

list(APPEND CMAKE_MODULE_PATH ${MELONDS_DIR}/cmake)
find_package(PkgConfig REQUIRED)  # net-utils looks up libenet with pkg-config
add_subdirectory(${MELONDS_DIR}/src/net net EXCLUDE_FROM_ALL)

add_executable(melonds-headless main.cpp platform.cpp)
target_link_libraries(melonds-headless PRIVATE core net-utils pthread)
# melonDS sets these per-directory; the runner must see the same class layouts
target_compile_definitions(melonds-headless PRIVATE GDBSTUB_ENABLED ARCHITECTURE_x86_64=1)
