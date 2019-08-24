# Locate the libpng library
#
# This module defines the following variables:
#
# LIBPNG_LIBRARY the name of the library;
# LIBPNG_INCLUDE_DIR where to find glfw include files.
# LIBPNG_FOUND true if both the GLFW3_LIBRARY and GLFW3_INCLUDE_DIR have been found.

set( _LIBPNG_HEADER_SEARCH_DIRS
	"/usr/include"
	"/usr/local/include"
	"${CMAKE_SOURCE_DIR}/includes"
	"C:/Program Files (x86)/glfw/include" )
set( _LIBPNG_LIB_SEARCH_DIRS
	"/usr/local/lib"
	"/usr/lib"
	"${CMAKE_SOURCE_DIR}/lib" )

# Search for the header
FIND_PATH(LIBPNG_INCLUDE_DIR png.h PATHS ${_LIBPNG_HEADER_SEARCH_DIRS} )

# Search for the library
FIND_LIBRARY(LIBPNG_LIBRARY NAMES libpng libpng12 PATHS ${_LIBPNG_LIB_SEARCH_DIRS} )

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBPNG DEFAULT_MSG LIBPNG_LIBRARY LIBPNG_INCLUDE_DIR)
