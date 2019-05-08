cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(TARGET deep_mimic_convert)
set(SOURCES main.cpp)
project(${TARGET})
add_executable(${TARGET} ${SOURCES})


##### BEGIN CONFIGURATION #####

if (WIN32)

  set(BULLET_ROOT_WIN32 "C:/Users/Hadas/Desktop/DeepMimic/deps/bullet3")

  set(BULLET_LIBRARIES_WIN32_DEBUG
      "${BULLET_ROOT_WIN32}/bin/BulletCollision_vs2010_x64_debug.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletDynamics_vs2010_x64_debug.lib"
      "${BULLET_ROOT_WIN32}/bin/LinearMath_vs2010_x64_debug.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletFileLoader_vs2010_x64_debug.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletWorldImporter_vs2010_x64_debug.lib")

  set(BULLET_LIBRARIES_WIN32_RELEASE
      "${BULLET_ROOT_WIN32}/bin/BulletCollision_vs2010_x64_release.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletDynamics_vs2010_x64_release.lib"
      "${BULLET_ROOT_WIN32}/bin/LinearMath_vs2010_x64_release.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletFileLoader_vs2010_x64_release.lib"
      "${BULLET_ROOT_WIN32}/bin/BulletWorldImporter_vs2010_x64_release.lib")

   set(BULLET_INCLUDE_DIRS "${BULLET_ROOT_WIN32}/src;${BULLET_ROOT_WIN32}/Extras/Serialize")
   set(BULLET_LIBRARIES ${BULLET_LIBRARIES_WIN32_DEBUG})

endif()

if (APPLE)

   find_package(BULLET REQUIRED)

   set(BULLET_EXTRA_LIBS_APPLE_RELEASE "/usr/local/lib/libBulletWorldImporter.dylib")

   set(BULLET_LIBRARIES "${BULLET_LIBRARIES};${BULLET_EXTRA_LIBS_APPLE_RELEASE}")

   find_package(jsoncpp REQUIRED)

   get_target_property(JSONCPP_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

endif()

##### END CONFIGURATION #####

TARGET_include_directories(${TARGET}
    PRIVATE
    ${DEFAULT_INCLUDE_DIRECTORIES}
    ${BULLET_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

TARGET_link_libraries(${TARGET}
    PRIVATE
    ${DEFAULT_LIBRARIES}
    ${BULLET_LIBRARIES}
    jsoncpp_lib
)

set_target_properties(${TARGET} PROPERTIES
    CXX_STANDARD 17
    CXX_EXTENSIONS OFF)
