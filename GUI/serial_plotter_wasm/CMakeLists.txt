cmake_minimum_required(VERSION 3.21)

project(serial_plotter_wasm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)
qt_standard_project_setup()

qt_add_executable(serial_plotter_wasm
    src/main.cpp
    src/plotwindow.cpp
    src/serialbridge.cpp
)

target_link_libraries(serial_plotter_wasm PRIVATE
    Qt6::Widgets
    Qt6::Charts
)

# Note: we intentionally do NOT set -sEXPORTED_RUNTIME_METHODS here. Qt sets it
# on its own link line and the last value wins, so ours would be silently
# dropped. The Web Serial glue in src/serialbridge.cpp therefore avoids ccall
# and talks to C++ through EMSCRIPTEN_KEEPALIVE exports + HEAPU8 instead.
