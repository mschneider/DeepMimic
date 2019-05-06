cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(target deep_mimic_convert)
set(sources main.cpp)

project(${target})
add_executable(${target} ${sources})

find_package(BULLET)

target_include_directories(${target}
    PRIVATE
    ${DEFAULT_INCLUDE_DIRECTORIES}
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(${target}
    PRIVATE
    ${DEFAULT_LIBRARIES}
    ${BULLET_LIBRARIES}
    "/usr/local/lib/libBulletWorldImporter.dylib"
)
