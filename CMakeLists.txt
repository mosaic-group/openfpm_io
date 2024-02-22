cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(openfpm_io LANGUAGES C CXX)

if(HDF5_FOUND)
	if (HDF5_IS_PARALLEL)
		set(DEFINE_HAVE_HDF5 "#define HAVE_HDF5")
	else()
		message( STATUS "HDF5 found ${HDF5_INCLUDE_DIRS} does not have parallel support, OpenFPM require it" )
	endif()
else()
	message( FATAL_ERROR "HDF5 with parallel support is required in order to install OpenFPM" )
endif()

if(TINYOBJLOADER_FOUND)
        set(DEFINE_HAVE_TINYOBJLOADER "#define HAVE_TINYOBJLOADER 1")
endif()


include_directories(SYSTEM ${MPI_INCLUDE_PATH})

add_subdirectory (../openfpm_vcluster openfpm_vcluster)

# collect all configs and write to config.h before compiling src
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/config/config_cmake.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/config/config.h)
add_subdirectory (src)

get_directory_property(hasParent PARENT_DIRECTORY)
if(hasParent)
	set(DEFINE_HAVE_TINYOBJLOADER ${DEFINE_HAVE_TINYOBJLOADER} CACHE INTERNAL "")
	set(DEFINE_HAVE_HDF5 ${DEFINE_HAVE_HDF5} CACHE INTERNAL "")
endif()

