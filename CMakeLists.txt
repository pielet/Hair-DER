cmake_minimum_required (VERSION 3.4.1)

project (Hair_DER)

# finds all files with a given extension
macro (append_files files ext)
  foreach (dir ${ARGN})
    file (GLOB _files "${dir}/*.${ext}")
    list (APPEND ${files} ${_files})
  endforeach (dir)
endmacro (append_files)

include_directories (
	${PROJECT_SOURCE_DIR}/include/rapidxml
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src/DER
)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# add libraries
find_package (TBB REQUIRED)
if (TBB_FOUND)
  	include_directories (${TBB_INCLUDE_DIRS})
  	set (LIBWETHAIR_LIBRARIES ${LIBWETHAIR_LIBRARIES} ${TBB_LIBRARIES})
else (TBB_FOUND)
  	message (SEND_ERROR "Unable to locate TBB")
endif (TBB_FOUND)

find_package (OpenGL REQUIRED)
if (OPENGL_FOUND)
  	include_directories (${OPENGL_INCLUDE_DIR})
  	set (LIBWETHAIR_LIBRARIES ${LIBWETHAIR_LIBRARIES} ${OPENGL_LIBRARIES})
else (OPENGL_FOUND)
  	message (SEND_ERROR "Unable to locate OpenGL")
endif (OPENGL_FOUND)

find_package(GLEW REQUIRED)
if (GLEW_FOUND)
  	include_directories(${GLEW_INCLUDE_DIRS})
  	set (LIBWETHAIR_LIBRARIES ${LIBWETHAIR_LIBRARIES} ${GLEW_LIBRARIES})
else (GLEW_FOUND)
  	message (SEND_ERROR "Unable to locate GLEW")
endif()

find_package(GLFW3 REQUIRED)
if (GLFW3_FOUND)
  	include_directories(${GLFW3_INCLUDE_DIR})
  	set (LIBWETHAIR_LIBRARIES ${LIBWETHAIR_LIBRARIES} ${GLFW3_LIBRARY})
else (GLFW3_FOUND)
  message (SEND_ERROR "Unable to locate GLFW3")
endif()

find_package(GLM REQUIRED)
if (GLM_FOUND)
	include_directories(${GLM_INCLUDE_DIRS})
else (GLM_FOUND)
	message(SEND_ERROR "Unable to locate GLM, GLM_INCLUDE_DIRS=${GLM_INCLUDE_DIRS}")
endif()

find_package (ANTTWEAKBAR REQUIRED)
if (ANTTWEAKBAR_FOUND)
	include_directories( ${ANT_TWEAK_BAR_INCLUDE_DIR} )
	set (LIBWETHAIR_LIBRARIES ${LIBWETHAIR_LIBRARIES} ${ANT_TWEAK_BAR_LIBRARY})  
else (ANTTWEAKBAR_FOUND)
	message (SEND_ERROR "Unable to locate ANTTWEAKBAR")
endif (ANTTWEAKBAR_FOUND)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

append_files (SRC_LIST "cpp" src src/DER src/DER/Dependencies src/DER/Forces)
list (REMOVE_ITEM SRC_LIST ${PROJECT_SOURCE_DIR}/src/main.cpp)

add_library (DER SHARED ${SRC_LIST})
target_link_libraries(DER PUBLIC ${LIBWETHAIR_LIBRARIES} "-lpthread")

add_executable (main ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries (main PUBLIC DER)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_SOURCE_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}//assets )

