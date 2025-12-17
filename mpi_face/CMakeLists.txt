cmake_minimum_required(VERSION 2.8.3)

project(mpi_face)

# Set cpp 14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QT related
set(QT_USE_QTXML TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# face output needs Ogre
# sudo apt install libogre-1.9-dev 
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/OGRE/cmake/modules)
find_package(OGRE REQUIRED COMPONENTS RenderSystem_GL)
add_definitions( -DOGRE_PLUGIN_PATH="/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/OGRE-1.9.0")

# GUI visualization needs Qt5
# sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5x11extras5-dev
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib/aarch64-linux-gnu/cmake/Qt5)
find_package(
  Qt5 
  REQUIRED COMPONENTS 
  Core 
  Widgets 
  X11Extras
)

# Specify additional locations of header files
include_directories(
  include
  SYSTEM
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5X11Extras_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIR}
)


###########
## Build ##
###########

# Set build type and output location
set(CMAKE_BUILD_TYPE Release)	
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")

# Declare source files
set(mpi_face_SRC
  src/mpi_face.cpp
  src/jhcFaceApp.cpp
  src/jhcFestTTS.cpp
  src/jhcAnimHead.cpp
)

# Declare header files (needed to specify all include files for wrapping with QT5)
set(mpi_face_INC
  include/jhcFaceApp.h
  include/jhcFestTTS.h
  include/jhcAnimHead.h
)

QT5_WRAP_CPP(mpi_face_MOC_SRC ${mpi_face_INC})

# Make shared library (TTS + face) ------------------------
add_library(mpi_face SHARED
  ${mpi_face_SRC} 
  ${mpi_face_MOC_SRC}
)

# Required input libraries for shared lib
target_link_libraries(mpi_face
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${Qt5X11Extras_LIBRARIES}
  ${OGRE_LIBRARIES}
  pthread
)

# Make shared library (TTS only) --------------------------

add_library(mpi_spout SHARED
  src/mpi_spout.cpp
  src/jhcFestTTS.cpp
)

# Required input libraries for shared lib
target_link_libraries(mpi_spout
  pthread
)


