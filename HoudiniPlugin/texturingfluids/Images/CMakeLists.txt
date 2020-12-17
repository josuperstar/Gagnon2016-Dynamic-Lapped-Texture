cmake_minimum_required(VERSION 2.8)

include(RezBuild)

set(ProjectName Images)

############## SET ############
set(ProjectSource .)
#set(Set Set)
set(Math  Math)
#set(Eigen eigen)

project(${ProjectName})

set(OpenCV_DIR "/prod/tools/rd/opencv-3.1.0-noqt/share/OpenCV")


############# FIND PACKAGE ###############

find_package(OpenCV REQUIRED core highgui)     # OpenCV root path must be in CMAKE_PREFIX_PATH environment variable.
include_directories(${OpenCV_INCLUDE_DIRS})


################# SOURCE ##################

file(GLOB source ${ProjectSource}/*.h ${ProjectSource}/*.C ${ProjectSource}/*.cpp ${ProjectSource}/*.hpp ${ProjectSource}/Set/*.h ${ProjectSource}/Set/SpacialGrid.hpp ${ProjectSource}/Math/*.h)

########### INCLUDES #################

include_directories(${ProjectSource})
include_directories(${Math})
include_directories(${Eigen})


########## LINKS ######################
if (APPLE)
    set(LTYPE MODULE)
else (APPLE)
    set(LTYPE SHARED)
endif (APPLE)


# this tells cmake to buid the DSO
add_library(${ProjectName} ${LTYPE} ${source})

rez_install_files(
  #build/houdini-15/lib${ProjectName}.so
  build/lib${ProjectName}.so
  RELATIVE "build"
  DESTINATION .
)
