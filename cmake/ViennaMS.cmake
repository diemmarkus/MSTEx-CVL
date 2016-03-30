macro(MAKE_ViennaMS_exe)

MAKE_ViennaMS()

include_directories (
	${CMAKE_CURRENT_SOURCE_DIR}/DkViennaMS
)

file(GLOB DKVIENNAMS_SOURCES "DkViennaMS/*.cpp")
file(GLOB DKVIENNAMSS_HEADERS "DkViennaMS/*.h")

set(EXE_NAME ${CMAKE_PROJECT_NAME})
add_definitions(-DDK_DEBUG)
add_definitions(-DDK_SAVE_DEBUG)
add_definitions(-DDK_STANDALONE)

add_executable(${EXE_NAME} MACOSX_BUNDLE ${DKVIENNAMS_SOURCES} ${DKVIENNAMS_HEADERS})
target_link_libraries(${EXE_NAME} ${OpenCV_LIBS} ${DKCOREDLL_NAME} ${DKMODULEDLL_NAME})

add_dependencies(${EXE_NAME} ${DKCOREDLL_NAME} ${DKMODULEDLL_NAME})

IF (MSVC) # copy opencv dlls and change settings for different projects

		FOREACH(opencvlib ${OpenCV_LIBS})
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Release/${opencvlib}*.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/RelWithDebInfo)
			
			FILE(GLOB dllpath ${OpenCV_DIR}/bin/Debug/${opencvlib}*d.dll)
			FILE(COPY ${dllpath} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)
			message(STATUS "${opencvlib} copied")
		ENDFOREACH(opencvlib)
			
		SET(_moc ${CMAKE_CURRENT_BINARY_DIR}/GeneratedFiles)
ENDIF(MSVC)

endmacro(MAKE_ViennaMS_exe)

macro(MAKE_ViennaMS)

include_directories (
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/DkModule
	${CMAKE_CURRENT_SOURCE_DIR}/DkCore
)

file(GLOB DKCORE_SOURCES "DkCore/*.cpp")
file(GLOB DKCORE_HEADERS "DkCore/*.h")

file(GLOB DKMODULE_SOURCES "DkModule/*.cpp")
file(GLOB DKMODULE_HEADERS "DkModule/*.h")

set(DKCOREDLL_NAME libDkCore)
set(DKCOPRELIB_NAME optimized ${DKCOREDLL_NAME}.lib debug ${DKCOREDLL_NAME}d.lib)

set(DKMODULEDLL_NAME libDkModule)
set(DKMODULELIB_NAME optimized ${DKMODULEDLL_NAME}.lib debug ${DKMODULEDLL_NAME}d.lib)

link_directories(${OpenCV_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/libs)

#DkCore
add_library(${DKCOREDLL_NAME} SHARED ${DKCORE_SOURCES} ${DKCORE_HEADERS})
target_link_libraries(${DKCOREDLL_NAME} ${OpenCV_LIBS})
set_target_properties(${DKCOREDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_CORE_EXPORTS  -DDK_DEBUG")
set_target_properties(${DKCOREDLL_NAME} PROPERTIES DEBUG_OUTPUT_NAME ${DKCOREDLL_NAME}d)

#DkModule
add_library(${DKMODULEDLL_NAME} SHARED ${DKMODULE_SOURCES} ${DKMODULE_HEADERS})
target_link_libraries(${DKMODULEDLL_NAME} ${OpenCV_LIBS} ${DKCOREDLL_NAME})
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES COMPILE_FLAGS "-DDK_MODULE_EXPORTS  -DDK_DEBUG")
set_target_properties(${DKMODULEDLL_NAME} PROPERTIES DEBUG_OUTPUT_NAME ${DKMODULEDLL_NAME}d)

if (ENABLE_PLUGIN)
	add_custom_command(TARGET ${DKCOREDLL_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${DKCOREDLL_NAME}> ${NOMACS_BUILD_DIRECTORY}/$(CONFIGURATION)/plugins/)
	add_custom_command(TARGET ${DKMODULEDLL_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${DKMODULEDLL_NAME}> ${NOMACS_BUILD_DIRECTORY}/$(CONFIGURATION)/plugins/)
endif (ENABLE_PLUGIN)

add_dependencies(${DKMODULEDLL_NAME} ${DKCOREDLL_NAME})

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
	SET_TARGET_PROPERTIES(${EXE_NAME} PROPERTIES LINK_FLAGS -fopenmp)
ENDIF()

set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

endmacro(MAKE_ViennaMS)
