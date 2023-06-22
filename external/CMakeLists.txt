# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GLEW
set(EXT_GLEW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/glew")
set(EXT_GLEW_INCLUDE_DIR "${EXT_GLEW_DIR}/include")
set(BUILD_UTILS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory("${EXT_GLEW_DIR}/build/cmake")
target_include_directories(glew_s PUBLIC
                           "$<BUILD_INTERFACE:${EXT_GLEW_INCLUDE_DIR}>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GLFW
set(EXT_GLFW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/glfw")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_GLFW_DIR})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# imgui
set(EXT_IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")
set(EXT_IMGUI_SOURCES
    "${EXT_IMGUI_DIR}/imgui.cpp"
    "${EXT_IMGUI_DIR}/imgui_draw.cpp"
    "${EXT_IMGUI_DIR}/imgui_widgets.cpp"
    "${EXT_IMGUI_DIR}/imgui_demo.cpp"
    "${EXT_IMGUI_DIR}/imgui_tables.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_opengl2.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)
add_library(imgui STATIC ${EXTERNAL_IMGUI_HEADERS} ${EXT_IMGUI_SOURCES})
target_include_directories(imgui PUBLIC
                           "$<BUILD_INTERFACE:${EXT_IMGUI_DIR};${EXT_IMGUI_DIR}/backends>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# VCGlib
set(EXT_VCGLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vcglib)
add_subdirectory(${EXT_VCGLIB_DIR})