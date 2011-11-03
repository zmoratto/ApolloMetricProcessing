# Make sure linker and compiler can find their way
include_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
include_directories(${VISIONWORKBENCH_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${FLANN_INCLUDE_DIRS})

set(VISIONWORKBENCH_BASE_LIBRARIES
  ${Boost_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${VISIONWORKBENCH_CORE_LIBRARY}
  ${VISIONWORKBENCH_MATH_LIBRARY}
  ${VISIONWORKBENCH_IMAGE_LIBRARY}
  ${VISIONWORKBENCH_FILEIO_LIBRARY}
  )

set(APOLLO_USED_LIBS
  ${VISIONWORKBENCH_BASE_LIBRARIES}
  ${FLANN_LIBRARIES}
  ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
  ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
  ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
  ${VISIONWORKBENCH_CAMERA_LIBRARY}
  ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
  )
add_apollo_tool( nvm_to_bundlevis nvm_to_bundlevis.cc )

set(APOLLO_USED_LIBS
  ${VISIONWORKBENCH_BASE_LIBRARIES}
  ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
  ${VISIONWORKBENCH_CAMERA_LIBRARY}
  )
add_apollo_tool( nvm_refine_tri nvm_refine_tri.cc )
add_apollo_tool( nvm_error nvm_error.cc )

if (StereoPipeline_FOUND AND QT_FOUND )
  include_directories(${StereoPipeline_INCLUDE_DIRS})
  include_directories(${ISIS_INCLUDE_DIRS})
  include_directories(${CSPICE_INCLUDE_DIRS})

  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${StereoPipeline_LIBRARIES}
    ${ISIS_LIBRARIES}
    ${CSPICE_LIBRARIES}
    ${QT_LIBRARIES}
    ${VISIONWORKBENCH_CAMERA_LIBRARY}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
    ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
    )

  add_apollo_tool( isis_cnet_to_nvm isis_cnet_to_nvm.cc )
  if (HAVE_BOOST_POLYGON_H)
    add_apollo_tool( obalog_brief_nvm obalog_brief_nvm.cc )
  endif()
  add_apollo_tool( nvm_to_isis_adjust nvm_to_isis_adjust.cc )

  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${VISIONWORKBENCH_CAMERA_LIBRARY}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
    ${StereoPipeline_CORE_LIBRARY}
    )
  add_apollo_tool( nvm_build nvm_build.cc )
endif()
