macro(NMC_POLICY)
	
endmacro(NMC_POLICY)

# Searches for Qt with the required components
macro(NMC_FINDQT)
	
	set(CMAKE_AUTOMOC ON)
	set(CMAKE_AUTORCC ON)
	set(CMAKE_INCLUDE_CURRENT_DIR ON)
	if(NOT QT_QMAKE_EXECUTABLE)
		find_program(QT_QMAKE_EXECUTABLE NAMES "qmake" "qmake-qt5" "qmake.exe")
	endif()
	if(NOT QT_QMAKE_EXECUTABLE)
		message(FATAL_ERROR "you have to set the path to the Qt5 qmake executable")
	endif()
	message(STATUS "QMake found: path: ${QT_QMAKE_EXECUTABLE}")
	GET_FILENAME_COMPONENT(QT_QMAKE_PATH ${QT_QMAKE_EXECUTABLE} PATH)
	set(QT_ROOT ${QT_QMAKE_PATH}/)
	SET(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${QT_QMAKE_PATH}\\..\\lib\\cmake\\Qt5)
	find_package(Qt5 REQUIRED Widgets Network LinguistTools PrintSupport Concurrent Gui)
	if (NOT Qt5_FOUND)
		message(FATAL_ERROR "Qt5Widgets not found. Check your QT_QMAKE_EXECUTABLE path and set it to the correct location")
	endif()
	add_definitions(-DQT5)
endmacro(NMC_FINDQT)

macro(NMC_FIND_OPENCV)
	SET(OpenCV_LIBS "")
	if (PKG_CONFIG_FOUND) # not sure: pkgconfig is needed for old linux  with old old opencv systems
	 pkg_check_modules(OpenCV  opencv>=2.1.0)
	 SET(OpenCV_LIBS ${OpenCV_LIBRARIES})
	endif(PKG_CONFIG_FOUND)
	IF (OpenCV_LIBS STREQUAL "") 
		find_package(OpenCV REQUIRED core imgproc)
	ENDIF()
	IF (NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV not found. It's mandatory when used with ENABLE_RAW enabled") 
	ELSE()
		add_definitions(-DWITH_OPENCV)
	ENDIF()
endmacro(NMC_FIND_OPENCV)

macro(NMC_PREPARE_PLUGIN)
	
	MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
	
	if(NOT NOMACS_VARS_ALREADY_SET) # is set when building nomacs and plugins at the sime time with linux
	 
		find_package(nomacs)
	 
		if(NOT NOMACS_FOUND)
			SET(NOMACS_BUILD_DIRECTORY "NOT_SET" CACHE PATH "Path to the nomacs build directory")
			IF (${NOMACS_BUILD_DIRECTORY} STREQUAL "NOT_SET")
				MESSAGE(FATAL_ERROR "You have to set the nomacs build directory")
			ENDIF()
		endif()
		SET(NOMACS_PLUGIN_INSTALL_DIRECTORY ${CMAKE_SOURCE_DIR}/install CACHE PATH "Path to the plugin install directory for deploying")
  		
	endif(NOT NOMACS_VARS_ALREADY_SET)
 
	if (CMAKE_BUILD_TYPE STREQUAL "debug" OR CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "DEBUG")
		message(STATUS "A debug build. -DDEBUG is defined")
		add_definitions(-DDEBUG)
		ADD_DEFINITIONS(-DQT_NO_DEBUG)
	elseif (NOT MSVC) # debug and release need qt debug outputs on windows
		message(STATUS "A release build (non-debug). Debugging outputs are silently ignored.")
#		add_definitions(-DQT_NO_DEBUG_OUTPUT)
	endif ()
 
endmacro(NMC_PREPARE_PLUGIN)

# you can use this NMC_CREATE_TARGETS("myAdditionalDll1.dll" "myAdditionalDll2.dll")
macro(NMC_CREATE_TARGETS)

	set(ADDITIONAL_DLLS ${ARGN})
		
	list(LENGTH ADDITIONAL_DLLS NUM_ADDITONAL_DLLS) 
	if( ${NUM_ADDITONAL_DLLS} GREATER 0) 
		foreach(DLL ${ADDITIONAL_DLLS})
			message(STATUS "extra_macro_args: ${DLL}")
		endforeach()
	endif()
	
	if(DEFINED GLOBAL_READ_BUILD)
		message(STATUS "project name: ${NOMACS_PROJECT_NAME}")
		add_dependencies(${PROJECT_NAME} ${NOMACS_PROJECT_NAME})
	else()
		# global build automatically puts the dll in the correct directory
		if(MSVC) # copy files on Windows in the correct directory
			# TODO copy all rdf opencv dlls to the nomacs target
			
			message(STATUS "opencv dlls: ${RDF_OPENCV_BINARIES}")
			
			set(BINS ${RDF_BINARIES} ${RDF_OPENCV_BINARIES})
			foreach(CUR_BIN ${BINS})
				string(REGEX MATCHALL ".*Debug.*" matches ${CUR_BIN})
				if(matches)
					file(COPY ${matches} DESTINATION ${NOMACS_BUILD_DIRECTORY}/Debug)
					add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${matches} ${NOMACS_BUILD_DIRECTORY}/Debug)
					add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${matches} ${NOMACS_BUILD_DIRECTORY}/RelWithDebInfo)
				endif()
				string(REGEX MATCHALL ".*Release.*" matches ${CUR_BIN})
				if(matches)
					add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${matches} ${NOMACS_BUILD_DIRECTORY}/Release)

				endif()			
			endforeach()
		else()
			foreach(CUR_BIN ${RDF_BINARIES})
				add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CUR_BIN} ${NOMACS_BUILD_DIRECTORY}/)
			endforeach()
		endif(MSVC)
	
	endif()
	
	
	if(MSVC)
		file(GLOB RDM_AUTOMOC "${CMAKE_BINARY_DIR}/*_automoc.cpp")
		source_group("Generated Files" FILES ${PLUGIN_RCC} ${RDM_QM} ${RDF_AUTOMOC})
		
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E make_directory ${NOMACS_BUILD_DIRECTORY}/$(CONFIGURATION)/plugins/)
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${NOMACS_BUILD_DIRECTORY}/$(CONFIGURATION)/plugins/)
		if(${NUM_ADDITONAL_DLLS} GREATER 0) 
			foreach(DLL ${ADDITIONAL_DLLS})
				add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND if 1==$<CONFIG:Debug> ${CMAKE_COMMAND} -E copy ${DLL} ${NOMACS_BUILD_DIRECTORY}/Debug/)
				add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND if 1==$<CONFIG:Release> ${CMAKE_COMMAND} -E copy ${DLL} ${NOMACS_BUILD_DIRECTORY}/Release/)
				add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND if 1==$<CONFIG:RelWithDebInfo> ${CMAKE_COMMAND} -E copy ${DLL} ${NOMACS_BUILD_DIRECTORY}/RelWithDebInfo/)
			endforeach()
		endif()		
		
		message(STATUS "${PROJECT_NAME} will be installed to: ${NOMACS_PLUGIN_INSTALL_DIRECTORY}")
		
		install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${NOMACS_PLUGIN_INSTALL_DIRECTORY}/packages/plugins.${PLUGIN_ARCHITECTURE}.${PROJECT_NAME}/data/nomacs-${PLUGIN_ARCHITECTURE}/plugins/ CONFIGURATIONS Release)
		install(FILES ${ADDITIONAL_DLLS} DESTINATION ${NOMACS_PLUGIN_INSTALL_DIRECTORY}/packages/plugins.${PLUGIN_ARCHITECTURE}.${PROJECT_NAME}/data/nomacs-${PLUGIN_ARCHITECTURE}/plugins/ CONFIGURATIONS Release)
		install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.xml DESTINATION ${NOMACS_PLUGIN_INSTALL_DIRECTORY}/packages/plugins.${PLUGIN_ARCHITECTURE}.${PROJECT_NAME}/meta CONFIGURATIONS Release)
	else()
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E make_directory ${NOMACS_BUILD_DIRECTORY}/plugins/)
		if(${NUM_ADDITONAL_DLLS} GREATER 0) 
			foreach(DLL ${ADDITIONAL_DLLS})
				add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${DLL} ${NOMACS_BUILD_DIRECTORY}/plugins/)
			endforeach()
		endif()
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${NOMACS_BUILD_DIRECTORY}/plugins/)
		install(TARGETS ${PROJECT_NAME} RUNTIME LIBRARY DESTINATION lib/nomacs-plugins)
	endif(MSVC)
endmacro(NMC_CREATE_TARGETS)

macro(NMC_PLUGIN_ID_AND_VERSION)
	list(LENGTH PLUGIN_JSON NUM_OF_FILES)
	if(NOT ${NUM_OF_FILES} EQUAL 1)
		message(FATAL_ERROR "${PROJECT_NAME} plugin has zero or more than one .json file")
	endif()
	file(STRINGS ${PLUGIN_JSON} line REGEX ".*\"PluginId\".*:")
	if(line)
		string(REGEX REPLACE ".*:\ +\"" "" PLUGIN_ID ${line})
		string(REGEX REPLACE "\".*" "" PLUGIN_ID ${PLUGIN_ID})
	else()
		message(FATAL_ERROR "${PROJECT_NAME}: PluginId missing in json file")
	endif()
	file(STRINGS ${PLUGIN_JSON} line REGEX ".*\"Version\".*:")
	if(line)
		string(REGEX REPLACE ".*:\ +\"" "" PLUGIN_VERSION ${line})
		string(REGEX REPLACE "\".*" "" PLUGIN_VERSION ${PLUGIN_VERSION})
	else()
		message(FATAL_ERROR "${PROJECT_NAME}: Version missing in json file")
	endif()
endmacro(NMC_PLUGIN_ID_AND_VERSION)
	
macro(NMC_GENERATE_PACKAGE_XML)
	set(JSON_FILE ${ARGN})

	# replace DATE_MODIFIED in json file to last cmake run
	file(STRINGS ${JSON_FILE} date_modified_line REGEX ".*\"DateModified\".*:")
	file(READ ${JSON_FILE} JSON_CONTENT)
	string(TIMESTAMP CURRENT_DATE "%Y-%m-%d")
	string(REPLACE "${date_modified_line}" "\t\"DateModified\"\t: \"${CURRENT_DATE}\"," JSON_CONTENT ${JSON_CONTENT})
	file(WRITE ${JSON_FILE} ${JSON_CONTENT})
	
	file(STRINGS ${JSON_FILE} line REGEX ".*\"PluginName\".*:")
	string(REGEX REPLACE ".*:\ +\"" "" PLUGIN_NAME ${line})
	string(REGEX REPLACE "\".*" "" PLUGIN_NAME ${PLUGIN_NAME})
	# message(STATUS "PLUGIN_NAME: ${PLUGIN_NAME}")

	file(STRINGS ${JSON_FILE} line REGEX ".*\"AuthorName\".*:")
	string(REGEX REPLACE ".*:\ +\"" "" AUTHOR_NAME ${line})
	string(REGEX REPLACE "\".*" "" AUTHOR_NAME ${AUTHOR_NAME})
	# message(STATUS "AUTHOR_NAME: ${AUTHOR_NAME}")
	
	file(STRINGS ${JSON_FILE} line REGEX ".*\"Company\".*:")
	string(REGEX REPLACE ".*:\ +\"" "" COMPANY_NAME ${line})
	string(REGEX REPLACE "\".*" "" COMPANY_NAME ${COMPANY_NAME})
	# message(STATUS "COMPANY_NAME: ${COMPANY_NAME}")
	
	file(STRINGS ${JSON_FILE} line REGEX ".*\"Tagline\".*:")
	string(REGEX REPLACE ".*:\ +\"" "" TAGLINE ${line})
	string(REGEX REPLACE "\".*" "" TAGLINE ${TAGLINE})
	# message(STATUS "TAGLINE: ${TAGLINE}")
	
	
	set(XML_CONTENT "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
	set(XML_CONTENT "${XML_CONTENT}<Package>\n")
	set(XML_CONTENT "${XML_CONTENT}\t<DisplayName>${PLUGIN_NAME} [${PLUGIN_ARCHITECTURE}]</DisplayName>\n")
	set(XML_CONTENT "${XML_CONTENT}\t<Description>${TAGLINE}</Description>\n")
	set(XML_CONTENT "${XML_CONTENT}\t<Version>${PLUGIN_VERSION}</Version>\n")
	set(XML_CONTENT "${XML_CONTENT}\t<ReleaseDate>${CURRENT_DATE}</ReleaseDate>\n")
	set(XML_CONTENT "${XML_CONTENT}\t<Default>false</Default>\n")
	set(XML_CONTENT "${XML_CONTENT}</Package>\n")
	
	file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/package.xml ${XML_CONTENT})
	
endmacro(NMC_GENERATE_PACKAGE_XML)

macro(NMC_GENERATE_USER_FILE)
	if(MSVC) # create user file only when using Visual Studio
		if(NOT EXISTS "${PROJECT_NAME}.vcxproj.user")
			configure_file(../cmake/project.vcxproj.user.in ${PROJECT_NAME}.vcxproj.user)
		endif()
	endif(MSVC)
endmacro(NMC_GENERATE_USER_FILE)