cmake_minimum_required(VERSION 2.8.11.2)
project(maptool)

## Compile as C++14
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)
add_definitions(-DLINUX)
# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
if(NOT DEFINED OGRE_PLUGIN_PATH)
  execute_process(COMMAND
    ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
    OUTPUT_VARIABLE OGRE_PLUGIN_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif(NOT DEFINED OGRE_PLUGIN_PATH)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})
add_definitions(-DOGRE_PLUGIN_PATH="${OGRE_PLUGIN_PATH}")

if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
            NAMES ${_lib}
            HINTS ${OGRE_OV_LIBRARY_DIRS}
            PATHS ${OGRE_OV_LIBRARY_DIRS}
    )

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
        CACHE FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)

find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

SET(CMAKE_AUTOMOC ON) #autocompile QOBJECT  xx.cpp - > moc_xxx.cpp
SET(CMAKE_AUTOUIC ON) #autocompile xx.ui -> ui_xx.h
SET(CMAKE_AUTORCC ON) #autocompile qrc 
SET(CMAKE_INCLUDE_CURRENT_DIR ON) # find moc_xxx ... files

find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
set(QT_VERSION ${Qt5Widgets_VERSION})
set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
set(qrc_files resources/maptool.qrc)


add_definitions(-DQT_NO_KEYWORDS)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

if(NOT OGRE_OV_LIBRARIES_ABS)
  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
endif()

if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
endif()

set(geo_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/libs)
set(geo_LIBRARIES ${PROJECT_SOURCE_DIR}/libs/geo/libecl_geo.a)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${OGRE_OV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  # ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    include/manage_map
    include/manage_map/FTP
    include/render_panel
    include/render_panel/ogre_helpers
    include/map_data  
    include/vmap_data
    include/dialog_view
    include/dialog_view/dialog_widget
    include/lanelet_data
    include/lanelet_data/lanelet2
    include/map_widget
    include/docks_vehicle_data
    include/docks_vehicle_data/docks_tool
    include/docks_vehicle_data/vehicle_mysql_info
    include/docks_vehicle_data/hdl_graph_slam
    include/tool
    ${geo_INCLUDE_DIRS}
    # ${OpenCV_LIBRARY_DIRS}
)

link_directories(
  "/usr/lib/x86_64-linux-gnu"
)

find_package(CURL REQUIRED)

set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/env_config.cpp)
configure_file(src/render_panel/ogre_helpers/env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

set(pqoases_LIB ${PROJECT_SOURCE_DIR}/libs/libqpOASES.a)
file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_CURRENT_BINARY_DIR} /resources/*.qrc)
file(GLOB_RECURSE SRCS "src/[a-zA-Z]*.cpp" "src/render_panel/ogre_helpers/[a-zA-Z]*.cpp" "src/lanelet_data/lanelet2/[a-zA-Z]*.cpp")
file(GLOB_RECURSE HDRS "include/[a-zA-Z]*.h" "include/lanelet_data/lanelet2/[a-zA-Z]*.h") 
file(GLOB_RECURSE DOTFILES ".*/*")
if (LIB_DOTFILES)
  list(REMOVE_ITEM SRCS ${LIB_DOTFILES})
  list(REMOVE_ITEM HDRS ${LIB_DOTFILES})
endif()

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${qrc_files})

add_executable(${PROJECT_NAME} ${SRCS} ${HDRS} ${ENV_CONFIG_FILE} ${QT_RESOURCES_CPP})
if(NOT WIN32)
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++14")
endif()
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${X11_X11_LIB} ${OGRE_OV_LIBRARIES_ABS} ${OPENGL_LIBRARIES} ${PCL_LIBRARIES}
                      ${pqoases_LIB} ${geo_LIBRARIES} ${CURL_LIBRARIES} ${OpenCV_LIBS}
                      Geographic boost_program_options -lmysqlclient)


