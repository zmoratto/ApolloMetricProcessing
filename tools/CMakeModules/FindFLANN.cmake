# Find FLANN

# This macro must define:
#  FLANN_FOUND             < Conditional
#  FLANN_INCLUDE_DIR       < Paths
#  FLANN_LIBRARIES         < Paths as well

find_path(FLANN_DIR
  NAMES  libflann_cpp.dylib
  HINTS  ${FLANN_INCLUDE_DIR}
  PATH_SUFFIXES lib lib64
)

find_PATH(FLANN_INCLUDE_DIR
  NAMES  flann/flann.hpp
  HINTS  ${FLANN_DIR}
  PATH_SUFFIXES inc include
)

# Deciding if FLANN was found
set(FLANN_INCLUDE_DIRS ${FLANN_INCLUDE_DIR})

if(FLANN_INCLUDE_DIR)
  set(FLANN_FOUND TRUE)
else(FLANN_INCLUDE_DIR)
  set(FLANN_FOUND FALSE)
endif(FLANN_INCLUDE_DIR)

FIND_LIBRARY(FLANN_LIBRARY
  NAMES flann_cpp
  HINTS ${FLANN_DIR}
  PATH_SUFFIXES lib lib64
)

set(FLANN_LIBRARIES ${FLANN_LIBRARY})