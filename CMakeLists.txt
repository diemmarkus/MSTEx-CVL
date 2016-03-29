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
PROJECT(ViennaMS)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

set(VIENNAMS_VERSION 0.0.l)
add_definitions(-DVIENNAMS_VERSION="${VIENNAMS_VERSION}")

# load paths from the user file if exists 
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
	include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "debug" OR CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    message(STATUS "A debug build. -DDEBUG is defined")
    add_definitions(-DDEBUG)
elseif (NOT MSVC) # debug and release need qt debug outputs on windows
    message(STATUS "A release build (non-debug). Debugging outputs are silently ignored.")
endif ()


#Set the custom CMake module directory where our include/lib finders are
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

IF(MSVC)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libs)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Debug)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Release)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)	
ENDIF(MSVC)

find_package(PkgConfig)

	          
# OpenCV
SET(OpenCV_REQUIRED_MODULES core imgproc highgui ml features2d flann objdetect video calib3d)
SET(OpenCV_LIBS "")
if (PKG_CONFIG_FOUND) # not sure: pkgconfig is needed for old linux  with old old opencv systems
	pkg_check_modules(OpenCV  opencv>=2.1.0)
	SET(OpenCV_LIBS ${OpenCV_LIBRARIES})
endif(PKG_CONFIG_FOUND)
IF (OpenCV_LIBS STREQUAL "") 
	find_package(OpenCV 3.0.0 REQUIRED ${OpenCV_REQUIRED_MODULES})
ENDIF()
IF (OpenCV_VERSION VERSION_LESS 2.4.0 AND OpenCV_FOUND AND MSVC) # OpenCV didn't allow to define packages before version 2.4.0 ... nomacs was linking against all libs even if they were not compiled -> error
	string(REGEX REPLACE "\\." "" OpenCV_SHORT_VERSION ${OpenCV_VERSION})
	SET(OpenCV_LIBS "debug;opencv_imgproc${OpenCV_SHORT_VERSION}d;optimized;opencv_imgproc${OpenCV_SHORT_VERSION};debug;opencv_core${OpenCV_SHORT_VERSION}d;optimized;opencv_core${OpenCV_SHORT_VERSION};")
ENDIF()

IF (NOT OpenCV_FOUND)
	message(FATAL_ERROR "OpenCV not found. It's mandatory when used with ENABLE_RAW enabled") 
ELSE()
	add_definitions(-DWITH_OPENCV -DHAVE_EXIV2_HPP)
ENDIF()

include_directories (
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/DkViennaMS
	${CMAKE_CURRENT_SOURCE_DIR}/DkModule
	${CMAKE_CURRENT_SOURCE_DIR}/DkCore
)

file(GLOB DKVIENNAMS_SOURCES "DkViennaMS/*.cpp")
file(GLOB DKVIENNAMSS_HEADERS "DkViennaMS/*.h")


file(GLOB DKCORE_SOURCES "DkCore/*.cpp")
file(GLOB DKCORE_HEADERS "DkCore/*.h")

file(GLOB DKMODULE_SOURCES "DkModule/*.cpp")
file(GLOB DKMODULE_HEADERS "DkModule/*.h")

set(DKCOREDLL_NAME libDkCore)
set(DKCOPRELIB_NAME optimized ${DKCOREDLL_NAME}.lib debug ${DKCOREDLL_NAME}d.lib)

set(DKMODULEDLL_NAME libDkModule)
set(DKMODULELIB_NAME optimized ${DKMODULEDLL_NAME}.lib debug ${DKMODULEDLL_NAME}d.lib)

set(EXE_NAME ${CMAKE_PROJECT_NAME})
add_definitions(-DDK_DEBUG)
add_definitions(-DDK_SAVE_DEBUG)
link_directories(${OpenCV_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/libs)

add_executable(${EXE_NAME} MACOSX_BUNDLE ${DKVIENNAMS_SOURCES} ${DKVIENNAMS_HEADERS})

target_link_libraries(${EXE_NAME} ${OpenCV_LIBS} ${DKCOREDLL_NAME} ${DKMODULEDLL_NAME})

set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE")
set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
set_target_properties(${EXE_NAME} PROPERTIES IMPORTED_IMPLIB "")


SET(CMAKE_SHARED_LINKER_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE") # /subsystem:windows does not work due to a bug in cmake (see http://public.kitware.com/Bug/view.php?id=12566)
#DkCore
add_library(${DKCOREDLL_NAME} SHARED ${DKCORE_SOURCES} ${DKCORE_HEADERS})
target_link_libraries(${DKCOREDLL_NAME} ${OpenCV_LIBS})
# set_target_properties(${DKCOREDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_DEBUG") # todo: dk_debug hier ist nicht gut, weil mans nicht ausschalten kann für realy release")
set_target_properties(${DKCOREDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_CORE_EXPORTS  -DDK_DEBUG") # todo: dk_debug hier ist nicht gut, weil mans nicht ausschalten kann für realy release")
set_target_properties(${DKCOREDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKCOREDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKCOREDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_REALLYRELEASE ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKCOREDLL_NAME} PROPERTIES DEBUG_OUTPUT_NAME ${DKCOREDLL_NAME}d)



#DkModule
add_library(${DKMODULEDLL_NAME} SHARED ${DKMODULE_SOURCES} ${DKMODULE_HEADERS})
target_link_libraries(${DKMODULEDLL_NAME} ${OpenCV_LIBS} ${DKCOREDLL_NAME})
# set_target_properties(${DKMODULEDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_DEBUG") # todo: dk_debug hier ist nicht gut, weil mans nicht ausschalten kann für realy release")
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_MODULE_EXPORTS  -DDK_DEBUG") # todo: dk_debug hier ist nicht gut, weil mans nicht ausschalten kann für realy release")
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_REALLYRELEASE ${CMAKE_CURRENT_BINARY_DIR}/libs)
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES DEBUG_OUTPUT_NAME ${DKMODULEDLL_NAME}d)

add_dependencies(${EXE_NAME} ${DKCOREDLL_NAME} ${DKMODULEDLL_NAME})
add_dependencies(${DKMODULEDLL_NAME} ${DKCOREDLL_NAME})

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
	SET_TARGET_PROPERTIES(${EXE_NAME} PROPERTIES LINK_FLAGS -fopenmp)
ENDIF()

IF (MSVC) # copy opencv dlls and change settings for different projects


		FOREACH(opencvlib ${OpenCV_REQUIRED_MODULES})
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Release/opencv_${opencvlib}*.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
			
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Debug/opencv_${opencvlib}*d.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
		ENDFOREACH(opencvlib)
	
		SET(CMAKE_CONFIGURATION_TYPES "Debug;Release;ReallyRelease" CACHE STRING "limited configs" FORCE)
		add_definitions(/Zc:wchar_t-)
		SET(CMAKE_CXX_FLAGS_REALLYRELEASE "-W4 -O2")
		SET(CMAKE_EXE_LINKER_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE") # /subsystem:windows does not work due to a bug in cmake (see http://public.kitware.com/Bug/view.php?id=12566)
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE")
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
		
		SET(_moc ${CMAKE_CURRENT_BINARY_DIR}/GeneratedFiles)
ENDIF()


