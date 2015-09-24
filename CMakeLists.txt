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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


set(DKNOMACS_VERSION 0.0.l)
add_definitions(-DDKNOMACS_VERSION="${NOMACS_VERSION}")

# load paths from the user file if exists 
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
	include(${CMAKE_CURRENT_SOURCE_DIR}/CMakeUser.cmake)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "debug" OR CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "DEBUG")
    message(STATUS "A debug build. -DDEBUG is defined")
    add_definitions(-DDEBUG)
elseif (NOT MSVC) # debug and release need qt debug outputs on windows
    message(STATUS "A release build (non-debug). Debugging outputs are silently ignored.")
    add_definitions(-DQT_NO_DEBUG_OUTPUT)
endif ()


#Set the custom CMake module directory where our include/lib finders are
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

IF(MSVC)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libs)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Debug)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Release)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)	
ENDIF(MSVC)

#set(BUILD_EXAMPLES OFF CACHE BOOL "Build all examples")

# find the Qt
set(QT_MIN_VERSION "4.7.0")
set(QT_USE_GUI 1)
set(QT_USE_QTNETWORK 1)
find_package(Qt4 REQUIRED QtCore QtGui QtNetwork)
include(${QT_USE_FILE})

find_package(PkgConfig)

	          
# OpenCV
SET(OpenCV_REQUIRED_MODULES core imgproc highgui ml features2d nonfree flann objdetect video calib3d)
SET(OpenCV_LIBS "")
if (PKG_CONFIG_FOUND) # not sure: pkgconfig is needed for old linux  with old old opencv systems
	pkg_check_modules(OpenCV  opencv>=2.1.0)
	SET(OpenCV_LIBS ${OpenCV_LIBRARIES})
endif(PKG_CONFIG_FOUND)
IF (OpenCV_LIBS STREQUAL "") 
	find_package(OpenCV 2.1.0 REQUIRED ${OpenCV_REQUIRED_MODULES})
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

#find nomacs
# SET(NOMACS_BUILD_DIRECTORY "NOT_SET" CACHE PATH "Path to the nomacs build directory")
IF (${NOMACS_BUILD_DIRECTORY} STREQUAL "NOT_SET")
	MESSAGE(FATAL_ERROR "You have to set the nomacs build directory")
ENDIF()

	
include_directories (
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/DkViennaMS
	${CMAKE_CURRENT_SOURCE_DIR}/DkNomacs
	${CMAKE_CURRENT_SOURCE_DIR}/DkModule
	${CMAKE_CURRENT_SOURCE_DIR}/DkCore
	${CMAKE_SOURCE_DIR}/DkNomacs/extern2012
	${NOMACS_BUILD_DIRECTORY}/../ImageLounge/src
	${NOMACS_BUILD_DIRECTORY}/../exiv2-0.24/msvc2012-nomacs/include
	${NOMACS_BUILD_DIRECTORY}/../LibRaw-0.16.0/
)

file(GLOB DKNOMACS_SOURCES "DkNomacs/*.cpp")
file(GLOB DKNOMACS_HEADERS "DkNomcas/*.h")
file(GLOB DKNOMACS_MOCS "DkNomacs/*.h")
file(GLOB DKNOMACS_CONFIGS "DkNomacs/*.conf")

file(GLOB DKVIENNAMS_SOURCES "DkViennaMS/*.cpp")
file(GLOB DKVIENNAMSS_HEADERS "DkViennaMS/*.h")

file(GLOB DKCORE_SOURCES "DkCore/*.cpp")
file(GLOB DKCORE_HEADERS "DkCore/*.h")

file(GLOB DKMODULE_SOURCES "DkModule/*.cpp")
file(GLOB DKMODULE_HEADERS "DkModule/*.h")


# set (DKNOMACS_FORMS
        # src/nomacs.ui
# )

set (DKNOMACS_RESOURCES
        DkNomacs/dknomacs.qrc
)

# set(DKNOMACS_TRANSLATIONS
		# translations/nomacs_als.ts
# }


# QT4_ADD_TRANSLATION(DKNOMACS_QM ${DKNOMACS_TRANSLATIONS})
# QT4_WRAP_UI(DKNOMACS_UI ${DKNOMACS_FORMS})
QT4_WRAP_CPP(DKNOMACS_MOC_SRC ${DKNOMACS_MOCS})
QT4_ADD_RESOURCES(DKNOMACS_RCC ${DKNOMACS_RESOURCES})

if (MSVC) # some settings for visual studio
	SET(DKNOMACS_RC DkNomacs/DkNomacs.rc) #add resource file when compiling with MSVC 
else(MSVC)
	SET(DKNOMACS_RC "")
ENDIF(MSVC)

set(DKCOREDLL_NAME libDkCore)
set(DKCOPRELIB_NAME optimized ${DKCOREDLL_NAME}.lib debug ${DKCOREDLL_NAME}d.lib)

set(DKMODULEDLL_NAME libDkModule)
set(DKMODULELIB_NAME optimized ${DKMODULEDLL_NAME}.lib debug ${DKMODULEDLL_NAME}d.lib)

set(NOMACSDLL_NAME optimized libnomacs.lib debug libnomacsd.lib)

set(EXE_NAME ${CMAKE_PROJECT_NAME}-nomacs)
# set(EXE_NAME ${CMAKE_PROJECT_NAME})
add_definitions(-DDK_DEBUG)
add_definitions(-DDK_DEBUG)
add_definitions(-DDK_SAVE_DEBUG)
link_directories(${LIBRAW_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/libs)
add_executable(${EXE_NAME} MACOSX_BUNDLE ${DKNOMACS_SOURCES} ${DKNOMACS_HEADERS} ${DKNOMACS_MOC_SRC} ${DKNOMACS_TRANSLATIONS} ${DKNOMACS_FORMS} ${DKNOMACS_MOCS} ${DKNOMACS_RESOURCES} ${DKNOMACS_CONFIGS} ${DKNOMACS_RC} ${DKNOMACS_RCC})

# add_executable(${EXE_NAME} MACOSX_BUNDLE ${DKVIENNAMS_SOURCES} ${DKVIENNAMS_HEADERS})

target_link_libraries(${EXE_NAME} ${OpenCV_LIBS} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTNETWORK_LIBRARY} ${QT_QTMAIN_LIBRARY} ${DKCOREDLL_NAME} ${DKMODULEDLL_NAME} ${NOMACSDLL_NAME})

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

IF (MSVC) # copy qt dlls and change settings for different projects


		FOREACH(opencvlib ${OpenCV_REQUIRED_MODULES})
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Release/opencv_${opencvlib}*.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
			
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Debug/opencv_${opencvlib}*d.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
		ENDFOREACH(opencvlib)
		
		SET (QTLIBLIST QtCore QtGui QtNetwork)
		FOREACH(qtlib ${QTLIBLIST})
			GET_FILENAME_COMPONENT(QT_DLL_PATH_tmp ${QT_QMAKE_EXECUTABLE} PATH)
			FILE(COPY ${QT_DLL_PATH_tmp}/${qtlib}4.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
			FILE(COPY ${QT_DLL_PATH_tmp}/${qtlib}4.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
			FILE(COPY ${QT_DLL_PATH_tmp}/${qtlib}d4.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
		ENDFOREACH(qtlib)
		
		#copy nomacs files
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/libnomacs.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/libnomacs.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/libnomacsd.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
		
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/libraw.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/libraw.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/libraw.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)		

		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/zlib1.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/zlib1.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/zlib1d.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)		

		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/libexpat.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/libexpat.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/libexpat.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)		

		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/exiv2.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/exiv2.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/exiv2d.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)		
		
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/libs/libnomacsd.lib DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/libs)
		FILE(COPY ${NOMACS_BUILD_DIRECTORY}/libs/libnomacs.lib DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/libs)

		set (quazipFile "${NOMACS_BUILD_DIRECTORY}/Release/quazip.dll")
		
		IF (EXISTS quazipFile)
			FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Release/quazip.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
			FILE(COPY ${NOMACS_BUILD_DIRECTORY}/ReallyRelease/quazip.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ReallyRelease)
			FILE(COPY ${NOMACS_BUILD_DIRECTORY}/Debug/quazipd.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)		
		ENDIF()		
		
		SET(CMAKE_CONFIGURATION_TYPES "Debug;Release;ReallyRelease" CACHE STRING "limited configs" FORCE)
		add_definitions(/Zc:wchar_t-)
		SET(CMAKE_CXX_FLAGS_REALLYRELEASE "-W4 -O2 -DQT_NO_DEBUG_OUTPUT")
		SET(CMAKE_EXE_LINKER_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE") # /subsystem:windows does not work due to a bug in cmake (see http://public.kitware.com/Bug/view.php?id=12566)
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_REALLYRELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:WINDOWS /LARGEADDRESSAWARE")
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
		set_target_properties(${EXE_NAME} PROPERTIES LINK_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /SUBSYSTEM:CONSOLE /LARGEADDRESSAWARE")
		
		SET(_moc ${CMAKE_CURRENT_BINARY_DIR}/GeneratedFiles)
		SOURCE_GROUP("Generated Files" FILES ${DKNOMACS_MOC_SRC} ${DKNOMACS_RCC} ${DKNOMACS_UI} ${DKNOMACS_RC} ${DKNOMACS_QM})
		set_source_files_properties(${DKNOMACS_TRANSLATIONS} PROPERTIES HEADER_FILE_ONLY TRUE)
		SOURCE_GROUP("Translations" FILES ${DKNOMACS_TRANSLATIONS})

		#add config file
		SOURCE_GROUP("Configs" FILES ${DKNOMACS_CONFIGS})
ENDIF()


