cmake_minimum_required(VERSION 2.8.3)

project(hmore_face)

# set cpp 14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the default path for built executables to the "bin" directory
# INSTALL(CODE "FILE(MAKE_DIRECTORY \${ENV}\${CMAKE_INSTALL_PREFIX}${mydir})")
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Use OGRE from ROS
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/OGRE/cmake/modules)

# QT related
set(QT_USE_QTXML TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(catkin REQUIRED COMPONENTS  
  roscpp
  roslib
  std_msgs
  geometry_msgs
)

# for face output
find_package(OGRE REQUIRED COMPONENTS RenderSystem_GL)
add_definitions( -DOGRE_PLUGIN_PATH="/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/OGRE-1.9.0")

# for GUI visualization
find_package(
  Qt5 
  REQUIRED COMPONENTS 
  Core 
  Widgets 
  X11Extras
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#   message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5X11Extras_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIR}
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
  roscpp
  message_runtime
  std_msgs
  geometry_msgs
)


###########
## Build ##
###########

# set build type
set(CMAKE_BUILD_TYPE Release)	

# Declare source files
set(hmore_face_SRC
  src/main.cpp
  src/jhcFaceNode.cpp
  src/jhcFestTTS.cpp
  src/jhcAnimHead.cpp
)

# Declare header files (needed to specify all include files for wrapping with QT5)
set(hmore_face_INC
  include/jhcFaceNode.h
  include/jhcFestTTS.h
  include/jhcAnimHead.h
)

QT5_WRAP_CPP(hmore_face_MOC_SRC ${hmore_face_INC})

add_executable(hmore_face 
  ${hmore_face_SRC} 
  ${hmore_face_MOC_SRC}
)

add_dependencies(hmore_face 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hmore_face
  ${catkin_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5X11Extras_LIBRARIES}
  ${OGRE_LIBRARIES}
  pthread
  uuid
)

install(TARGETS hmore_face
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ptu.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#	 target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
