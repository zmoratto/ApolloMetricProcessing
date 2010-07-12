# Find CSPICE

# This macro must define:
#  CSPICE_FOUND             < Conditional
#  CSPICE_INCLUDE_DIR       < Paths
#  CSPICE_LIBRARIES         < Paths as well

find_path(CSPICE_DIR
  NAMES  lib/cspice.dylib
  HINTS  ${CSPICE_INCLUDE_DIR}
         ${ISISROOT}/3rdParty
)

find_PATH(CSPICE_INCLUDE_DIR
  NAMES  naif/SpiceCK.h
         naif/SpiceSPK.h
  HINTS  ${CSPICE_DIR}
         ${ISISROOT}/3rdParty
  PATH_SUFFIXES inc include
)

# Deciding if CSPICE was found
set(CSPICE_INCLUDE_DIRS ${CSPICE_INCLUDE_DIR})

if(CSPICE_INCLUDE_DIR)
  set(CSPICE_FOUND TRUE)
else(CSPICE_INCLUDE_DIR)
  set(CSPICE_FOUND FALSE)
endif(CSPICE_INCLUDE_DIR)

FIND_LIBRARY(CSPICE_LIBRARY
  NAMES cspice
  HINTS ${CSPICE_DIR}
  PATH_SUFFIXES lib lib64
)

set(CSPICE_LIBRARIES ${CSPICE_LIBRARY})