cmake_minimum_required(VERSION 3.10)
project(imgui_pid)

set(CMAKE_CXX_STANDARD 17)

# Voeg de resource file toe
set(RESOURCE_FILES resource.rc)

add_executable(imgui_pid
        main.cpp
        pid.cpp
        imgui/imgui.cpp
        imgui/imgui_draw.cpp
        imgui/imgui_tables.cpp
        imgui/imgui_widgets.cpp
        imgui/backends/imgui_impl_win32.cpp
        imgui/backends/imgui_impl_dx11.cpp
        ${RESOURCE_FILES}
)

target_include_directories(imgui_pid PRIVATE
        .
        imgui
        imgui/backends
)

target_link_libraries(imgui_pid
        d3d11
        dxgi
        dwmapi
        d3dcompiler
)

enable_language(RC)

set_target_properties(imgui_pid PROPERTIES
        WIN32_EXECUTABLE TRUE
)