set(SOURCES
    bound_actuation.cpp
    framework/app.cpp
    framework/debug.cpp
    framework/input.cpp
    "${EXTERNAL_DIR}/vcglib/wrap/gui/trackball.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/gui/trackmode.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/ply/plylib.cpp"
)

add_executable(${MAIN_EXE_TARGET} ${SOURCES})
target_include_directories(${MAIN_EXE_TARGET} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

include(FindOpenGL)

list(APPEND CMAKE_MODULE_PATH "/usr/lib64/cmake/OpenVDB/")
set(OpenVDB_INCLUDE_DIR "/usr/include/openvdb")
find_package(OpenVDB REQUIRED openvdb)

target_link_libraries(${MAIN_EXE_TARGET}
                      ${OPENGL_glu_LIBRARY}
                      glew_s
                      glfw
                      imgui
                      vcglib
                      OpenVDB::openvdb)