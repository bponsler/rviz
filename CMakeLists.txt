cmake_minimum_required(VERSION 3.5)
project(rviz)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
#find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(python_qt_binding REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(ros2_console REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(X11 REQUIRED)

if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

find_package(Boost
  REQUIRED
  COMPONENTS filesystem program_options signals system thread)

find_package(PkgConfig REQUIRED)

find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if(ASSIMP_FOUND)
  if(NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})  
else()
  message(STATUS
    "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()

pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
if(NOT OGRE_OV_FOUND)
  pkg_check_modules(OGRE_OV REQUIRED OGRE)
endif(NOT OGRE_OV_FOUND)

## Find OGRE Plugin path (not necessarily platform-independent, I guess)
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir
  OGRE
  OUTPUT_VARIABLE
  OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})

# find absolute path of ogre libraries.
# This is stored in the cache to save time on cmake re-run.
# Using absolute paths is necessary if pkg-config finds Ogre in a different
# location than the default.  This can happen when Ogre is built from source,
# or when 2 versions of Ogre are installed.  Using absolute paths ensures that
# components that link against Ogre (rviz and any packages depending on rviz)
# all find the same Ogre shared library.
if(NOT DEFINED OGRE_OV_LIBRARIES_ABS)
  unset(OGRE_OV_LIBRARIES_ABS_TMP)
  foreach(_lib ${OGRE_OV_LIBRARIES})
    set(OGRE_OV_LIB_TAG "OGRE_OV_RVIZ_LIB_${_lib}")

    find_library(${OGRE_OV_LIB_TAG}
      NAMES ${_lib}
      HINTS ${OGRE_OV_LIBRARY_DIRS}
      PATHS ${OGRE_OV_LIBRARY_DIRS})

    set(OGRE_OV_LIBRARIES_ABS_TMP ${OGRE_OV_LIBRARIES_ABS_TMP} ${${OGRE_OV_LIB_TAG}})
  endforeach(_lib)

  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES_ABS_TMP}
    CACHE
    FILEPATH "Pathname of library ${_lib}")
endif(NOT DEFINED OGRE_OV_LIBRARIES_ABS)

if(APPLE)
  find_library(Cocoa_LIBRARIES Cocoa)
  set(rviz_ADDITIONAL_LIBRARIES ${Cocoa_LIBRARIES})
endif()

find_package(OpenGL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

option(UseQt5 "UseQt5" ON)
if(UseQt5)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
else()
  find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
endif()

if(${tf2_VERSION} VERSION_LESS "1.11.3")
  add_definitions("-DRVIZ_USE_BOOST_SIGNAL_1")
endif()

find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
  REQUIRED)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES}) # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

if(NOT OGRE_OV_LIBRARIES_ABS)
  set(OGRE_OV_LIBRARIES_ABS ${OGRE_OV_LIBRARIES})
endif()

if(NOT UseQt5)
  include(${QT_USE_FILE})
endif()
add_definitions(-DQT_NO_KEYWORDS)

# Set the Qt version for use in the extras file.
if(UseQt5)
  set(rviz_QT_VERSION ${Qt5Widgets_VERSION})
else()
  set(rviz_QT_VERSION ${QTVERSION})
endif()

set(INCLUDE_DIRS src ${ament_cmake_INCLUDE_DIRS} ${angles_INCLUDE_DIRS}
  ${cmake_modules_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS} # ${image_transport_INCLUDE_DIRS}
  ${interactive_markers_INCLUDE_DIRS} ${laser_geometry_INCLUDE_DIRS}
  ${map_msgs_INCLUDE_DIRS} ${message_filters_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS} ${python_qt_binding_INCLUDE_DIRS}
  ${resource_retriever_INCLUDE_DIRS}
  ${ros2_console_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${rclpy_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${sensor_msgs_util_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${urdf_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS} ${pluginlib_INCLUDE_DIRS} ${OGRE_OV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${Qt5Gui_OPENGL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${X11_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${angles_LIBRARY_DIRS}
  ${cmake_modules_LIBRARY_DIRS} ${geometry_msgs_LIBRARY_DIRS}
  ${image_geometry_LIBRARY_DIRS} #${image_transport_LIBRARY_DIRS}
  ${interactive_markers_LIBRARY_DIRS} ${laser_geometry_LIBRARY_DIRS}
  ${map_msgs_LIBRARY_DIRS} ${message_filters_LIBRARY_DIRS}
  ${nav_msgs_LIBRARY_DIRS} ${python_qt_binding_LIBRARY_DIRS}
  ${resource_retriever_LIBRARY_DIRS}
  ${ros2_console_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${rclpy_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS} ${sensor_msgs_util_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS}
  ${std_srvs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS} ${tf2_ros_LIBRARY_DIRS} ${urdf_LIBRARY_DIRS}
  ${visualization_msgs_LIBRARY_DIRS} ${pluginlib_LIBRARY_DIRS} ${OGRE_OV_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS} ${Qt5Gui_OPENGL_LIBRARY_DIRS} ${Qt5Widgets_LIBRARY_DIRS}
  ${Qt5OpenGL_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS} ${X11_LIBRARY_DIRS}
)

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${angles_LIBRARIES} ${cmake_modules_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${image_geometry_LIBRARIES} #${image_transport_LIBRARIES}
  ${interactive_markers_LIBRARIES}
  ${laser_geometry_LIBRARIES} ${map_msgs_LIBRARIES} ${message_filters_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${python_qt_binding_LIBRARIES}
  ${resource_retriever_LIBRARIES} ${ros2_console_LIBRARIES}
  ${rclcpp_LIBRARIES} ${rclpy_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${sensor_msgs_util_LIBRARIES} ${std_msgs_LIBRARIES} ${std_srvs_LIBRARIES}
  ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES} ${urdf_LIBRARIES}
  ${visualization_msgs_LIBRARIES} ${pluginlib_LIBRARIES} ${OGRE_OV_LIBRARIES}
  ${Boost_LIBRARIES} ${Qt5Gui_OPENGL_LIBRARIES} ${Qt5Widgets_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES} ${EIGEN3_LIBRARIES} ${X11_LIBRARIES}
)

# This variable controls the target name of the default plugin library.
# It is used in the extras file (processed in caktin_package(...)) and the
# cmake code for the default plugin.
# There is a matching instance of this in the plugin_description.xml.
set(rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME rviz_default_plugin)

#### If gtk ends up being the best way to get the correct window
#### position under X11, this is how to compile it in.
#
# find_package(GTK2)
# include_directories(${GTK2_INCLUDE_DIRS})
# include_directories(/usr/include/gdk-pixbuf-2.0)

add_subdirectory(src)

install(DIRECTORY ogre_media DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons DESTINATION share/${PROJECT_NAME})
install(DIRECTORY images DESTINATION share/${PROJECT_NAME})
install(FILES default.rviz DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(angles)
ament_export_dependencies(cmake_modules)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(image_geometry)
#ament_export_dependencies(image_transport)
ament_export_dependencies(interactive_markers)
ament_export_dependencies(laser_geometry)
ament_export_dependencies(map_msgs)
ament_export_dependencies(message_filters)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(python_qt_binding)
ament_export_dependencies(resource_retriever)
ament_export_dependencies(ros2_console)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(sensor_msgs_util)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(urdf)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(X11)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
