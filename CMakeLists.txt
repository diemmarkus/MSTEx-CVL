# usage:
# cd nomacs/root/dir
# mkdir build
# cd build
# cmake ../ImageLounge -DCMAKE_INSTALL_PREFIX=../release -DCMAKE_BUILD_TYPE=debug (or release, by default)
#       on windows there should be -G "generator name" required as well...
# make
# make install
#
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(ViennaMS)

#include cmakes
include("cmake/Utils.cmake")
include("cmake/ViennaMS.cmake")

set(VIENNAMS_VERSION 0.0.l)
add_definitions(-DVIENNAMS_VERSION="${VIENNAMS_VERSION}")

# # load paths from the user file if exists 
# if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
	# include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
# endif()

if (CMAKE_BUILD_TYPE STREQUAL "debug" OR CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    message(STATUS "A debug build. -DDEBUG is defined")
    add_definitions(-DDEBUG)
elseif (NOT MSVC) # debug and release need qt debug outputs on windows
    message(STATUS "A release build (non-debug). Debugging outputs are silently ignored.")
endif ()

NMC_POLICY()

# ask for plugin to be built
OPTION (ENABLE_PLUGIN "Compile as nomacs Plugin" OFF)

NMC_FIND_OPENCV()
          
if (ENABLE_PLUGIN)

	NMC_PREPARE_PLUGIN()

	NMC_FINDQT()

	if(CMAKE_CL_64)
		SET(PLUGIN_ARCHITECTURE "x64")
	else()
		SET(PLUGIN_ARCHITECTURE "x86")
	endif()

	set(BUILDING_MULTIPLE_PLUGINS true)
	
	MAKE_ViennaMS()

	add_subdirectory(DkPlugin)
		
else(ENABLE_PLUGIN)
	
	MAKE_ViennaMS_exe()

endif (ENABLE_PLUGIN)		  

