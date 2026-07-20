cmake_minimum_required(VERSION 3.21)

project(serial_plotter_wasm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Emits build/<dir>/compile_commands.json, which the VS Code C/C++ extension
# reads to resolve the Qt headers (see .vscode/c_cpp_properties.json).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Charts)
qt_standard_project_setup()

qt_add_executable(serial_plotter_wasm
    src/main.cpp
    src/plotwindow.cpp
    src/serialbridge.cpp
    src/demosource.cpp
)

target_link_libraries(serial_plotter_wasm PRIVATE
    Qt6::Widgets
    Qt6::Charts
)

# Note: we intentionally do NOT set -sEXPORTED_RUNTIME_METHODS here. Qt sets it
# on its own link line and the last value wins, so ours would be silently
# dropped. The Web Serial glue in src/serialbridge.cpp therefore avoids ccall
# and talks to C++ through EMSCRIPTEN_KEEPALIVE exports + HEAPU8 instead.

if(EMSCRIPTEN)
    # Qt emits <target>.html; GitHub Pages needs index.html. Copying it as a
    # post-build step keeps the two in lockstep -- renaming by hand leaves a
    # stale loader behind on the next rebuild.
    #
    # .nojekyll stops GitHub Pages running the output through Jekyll, which
    # would drop any file or directory whose name begins with an underscore.
    add_custom_command(TARGET serial_plotter_wasm POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE_DIR:serial_plotter_wasm>/serial_plotter_wasm.html"
                "$<TARGET_FILE_DIR:serial_plotter_wasm>/index.html"
        COMMAND ${CMAKE_COMMAND} -E touch
                "$<TARGET_FILE_DIR:serial_plotter_wasm>/.nojekyll"
        COMMENT "Generating index.html + .nojekyll for GitHub Pages"
        VERBATIM
    )
endif()
