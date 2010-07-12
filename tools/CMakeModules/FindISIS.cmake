# Find ISIS

# This macro must define:
#  ISIS_FOUND             < Conditional
#  ISIS_INCLUDE_DIR       < Paths
#  ISIS_LIBRARIES         < Paths as well

find_path(ISIS_DIR
  NAMES  inc/Isis.h
         lib/libisis3.dylib
  HINTS  ${ISIS_INCLUDE_DIR}
         $ENV{ISISROOT}
)

find_PATH(ISIS_INCLUDE_DIR
  NAMES  Isis.h
  HINTS  ${ISIS_DIR}
         $ENV{ISISROOT}
  PATH_SUFFIXES inc
)

# Deciding if ISIS was found
set(ISIS_INCLUDE_DIRS ${ISIS_INCLUDE_DIR})

if(ISIS_INCLUDE_DIR)
  set(ISIS_FOUND TRUE)
else(ISIS_INCLUDE_DIR)
  set(ISIS_FOUND FALSE)
endif(ISIS_INCLUDE_DIR)

FIND_LIBRARY(ISIS_LIBRARY
  NAMES isis3 isis3.2 isis3.2.0
  HINTS ${ISIS_DIR}
  PATH_SUFFIXES lib lib64
)

set(ISIS_LIBRARIES ${ISIS_LIBRARY})