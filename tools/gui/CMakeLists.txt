include(FetchContent)

find_package(OpenGL REQUIRED)

if(UNIX AND NOT APPLE)
    find_package(X11 QUIET)
    if(NOT X11_FOUND OR
       NOT X11_Xrandr_FOUND OR
       NOT X11_Xinerama_FOUND OR
       NOT X11_Xcursor_FOUND OR
       NOT X11_Xi_FOUND)
        message(FATAL_ERROR
            "PIDPP_BUILD_GUI=ON requires X11 development packages for GLFW. "
            "On Ubuntu/Debian install: libgl1-mesa-dev libx11-dev libxrandr-dev "
            "libxinerama-dev libxcursor-dev libxi-dev."
        )
    endif()
endif()

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
if(UNIX AND NOT APPLE)
    set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
endif()
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.90.9
)

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)

FetchContent_MakeAvailable(imgui implot)

add_library(pidpp_gui_imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

target_compile_features(pidpp_gui_imgui PUBLIC cxx_std_20)
target_include_directories(pidpp_gui_imgui
    PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${implot_SOURCE_DIR}
)
target_link_libraries(pidpp_gui_imgui PUBLIC glfw OpenGL::GL)

add_executable(pidpp_control_lab main.cpp)
target_link_libraries(pidpp_control_lab PRIVATE pidpp::pidpp pidpp_gui_imgui)

if(PIDPP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(pidpp_gui_imgui PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
    target_link_options(pidpp_gui_imgui PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
    target_compile_options(pidpp_control_lab PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
    target_link_options(pidpp_control_lab PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
endif()
