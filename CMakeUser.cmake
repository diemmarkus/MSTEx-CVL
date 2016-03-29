# If you want to use prefix paths with cmake, copy and rename this file to CMakeUser.cmake
# Do not add this file to GIT!

# set your preferred Qt Library path
IF (CMAKE_CL_64)
	SET(CMAKE_PREFIX_PATH "D:/Qt/qt-everywhere-opensource-src-5.5.1-x64/bin/")
ELSE ()
	SET(CMAKE_PREFIX_PATH "D:/Qt/qt-everywhere-opensource-src-5.5.1-x86/bin/")
ENDIF ()

# set your preferred OpenCV Library path
IF (CMAKE_CL_64)
	SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/VSProjects/OpenCV3/build2015-x64")
ELSE ()
	SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/VSProjects/OpenCV3/build2015-x86")
ENDIF ()

# set your preferred HUpnp path
IF (CMAKE_CL_64)
	SET(NOMACS_BUILD_DIRECTORY ${NOMACS_BUILD_DIRECTORY} "C:/VSProjects/nomacs/build2015-x64")
ELSE ()
	SET(NOMACS_BUILD_DIRECTORY ${NOMACS_BUILD_DIRECTORY} "C:/VSProjects/nomacs/build2015-x86")
ENDIF ()
