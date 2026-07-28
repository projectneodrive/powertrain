cmake_minimum_required(VERSION 3.21)
project(hub_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts)

# Reuse the real production sources (desktop build: the Web Serial glue in
# serialbridge.cpp is #ifdef'd out, leaving the plain C++ path).
qt_add_executable(hub_test
    hub_test.cpp
    ../src/telemetryhub.cpp
    ../src/serialbridge.cpp
)
target_include_directories(hub_test PRIVATE ../src)
target_link_libraries(hub_test PRIVATE Qt6::Core)

qt_add_executable(liveplot_test
    liveplot_test.cpp
    ../src/liveplot.cpp
)
target_include_directories(liveplot_test PRIVATE ../src)
target_link_libraries(liveplot_test PRIVATE Qt6::Widgets Qt6::Charts)
