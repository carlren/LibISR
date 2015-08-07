# - Find OpenNI2
# This module defines
#  OpenNI_INCLUDE_DIR, where to find OpenNI include files
#  OpenNI_LIBRARIES, the libraries needed to use OpenNI
#  OpenNI_FOUND, If false, do not try to use OpenNI.
# also defined, but not for general use are
#  OpenNI_LIBRARY, where to find the OpenNI library.

message(STATUS ${OPEN_NI_ROOT})

find_path(OpenNI_INCLUDE_DIR OpenNI.h PATH "${OPEN_NI_ROOT}/Include")
find_library(OpenNI_LIBRARY OpenNI2 PATH "${OPEN_NI_ROOT}/Bin/x64-Release/")

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(OpenNI DEFAULT_MSG OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

if(OPENNI_FOUND)
  set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
endif()

mark_as_advanced(OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

