# Find Vision Workbench
#
# == Using Header-Only libraries from within Vision Workbench: ==
#
#  find_package( VisionWorkbench 2.0 )
#  if(VisionWorkbench_FOUND)
#     include_directories(${VisionWorkbench_INCLUDE_DIRS})
#     add_executable(foo foo.cc)
#  endif()
#
# == Using actual libraries from within Vision Workbench: ==
#
#  find_package( VisionWorkbench 2.0 COMPONENTS core image fileio camera ... )
#
#  if(VisionWorkbench_FOUND)
#     include_directories(${VisionWorkbench_INCLUDE_DIRS})
#     add_executable(foo foo.cc)
#     target_link_libraries(foo ${VisionWorkbench_LIBRARIES})
#  endif()
#
# ===========================================================
#
#  Copyright (c) 2009 Zachary Moratto
#
#  Redistribution AND use is allowed according to the terms
#  of the NEW BSD license.
#

# This macro must define:
#  VisionWorkbench_FOUND             < Conditional
#  VisionWorkbench_INCLUDE_DIR       < Paths
#  VisionWorkbench_LIBRARIES         < Paths as well

# Find Path



find_path(VisionWorkbench_DIR
  NAMES  include/vw/vw.h
         lib/libvwCore.dylib
  HINTS  ${VisionWorkbench_INCLUDE_DIR}
)

find_path(VisionWorkbench_INCLUDE_DIR
  NAMES  vw/vw.h
  HINTS  ${VisionWorkbench_DIR}
  PATH_SUFFIXES include
)

# Searching for each library that makes up a component
foreach(COMPONENT ${VisionWorkbench_FIND_COMPONENTS})
  string(TOUPPER ${COMPONENT} UPPERCOMPONENT)
  set( VisionWorkbench_${UPPERCOMPONENT}_LIBRARY "VisionWorkbench_${UPPERCOMPONENT}_LIBRARY-NOTFOUND")

  string(REGEX REPLACE "^(.)(.+)$" "\\1" first_half "${UPPERCOMPONENT}")
  string(REGEX REPLACE "^(.)(.+)$" "\\2" second_half "${COMPONENT}")

  set(LIBRARY_NAME "${first_half}${second_half}")
  if ( ${UPPERCOMPONENT} STREQUAL "VW" )
    set(LIBRARY_NAME "")
  elseif( ${UPPERCOMPONENT} STREQUAL "INTERESTPOINT" )
    set(LIBRARY_NAME "InterestPoint")
  elseif( ${UPPERCOMPONENT} STREQUAL "HDR" )
    set(LIBRARY_NAME "HDR")
  elseif( ${UPPERCOMPONENT} STREQUAL "FILEIO")
    set(LIBRARY_NAME "FileIO")
  endif()

  find_library( VisionWorkbench_${UPPERCOMPONENT}_LIBRARY
    NAMES  vw${LIBRARY_NAME}
    HINTS  ${VisionWorkbench_DIR}
    PATH_SUFFIXES lib lib64
    )

  if(VisionWorkbench_${UPPERCOMPONENT}_LIBRARY)
    set(VisionWorkbench_${UPPERCOMPONENT}_FOUND TRUE CACHE INTERNAL "If the VW ${UPPERCOMPONENT} library was found")
  endif()

endforeach(COMPONENT)

# Deciding if VW was found
set(VisionWorkbench_INCLUDE_DIRS ${VisionWorkbench_INCLUDE_DIR})

if(VisionWorkbench_INCLUDE_DIR)
  set(VisionWorkbench_FOUND TRUE)
else(VisionWorkbench_INCLUDE_DIR)
  set(VisionWorkbench_FOUND FALSE)
endif(VisionWorkbench_INCLUDE_DIR)

# Closing Messages
if(VisionWorkbench_FOUND)
  message(STATUS "Found the following VisionWorkbench libraries:")
  foreach( COMPONENT ${VisionWorkbench_FIND_COMPONENTS})
    string(TOUPPER ${COMPONENT} UPPERCOMPONENT )
    if ( VisionWorkbench_${UPPERCOMPONENT}_FOUND )
      message(STATUS "  ${COMPONENT}")
      set(VisionWorkbench_LIBRARIES ${VisionWorkbench_LIBRARIES} ${VisionWorkbench_${UPPERCOMPONENT}_LIBRARY})
    endif()
  endforeach()
else(VisionWorkbench_FOUND)
  message(SEND_ERROR "Unable to find requested VisionWorkbench libraries")
endif(VisionWorkbench_FOUND)

#find_path(VisionWorkbench_INCLUDE_DIR vw.h
#  PATHS    /usr/local
#           /opt/local
#           /usr
#  PATH_SUFFIXES include
#)

#find_library(VisionWorkbench_LIBRARY vw
#  PATHS    /usr/local
#           /opt/local
#           /usr
#  PATH_SUFFIXES lib64 lib
#)

