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
	/usr/local/include
	/usr/include
	./include/rapidxml
	./src/DER
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

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

append_files (SRC_LIST "cpp" src src/DER src/DER/Dependencies src/DER/Forces src/OpenGL)
list (REMOVE_ITEM SRC_LIST ./src/main.cpp)
add_library (DER SHARED ${SRC_LIST})

add_executable (main ./src/main.cpp)
target_link_libraries (main DER)
target_link_libraries (main ${LIBWETHAIR_LIBRARIES} "-lpthread")
