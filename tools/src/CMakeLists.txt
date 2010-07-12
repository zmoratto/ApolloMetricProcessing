# Make sure linker and compiler can find their way
include_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
link_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
message(STATUS "Adding ${ApolloMetricProcessing_SOURCE_DIR}/src")

# reduce_match
# - Used to reduce matches post process
add_executable( reduce_match reduce_match.cc equalization.cc )
target_link_libraries( reduce_match
                       ${Boost_LIBRARIES}
                       ${VisionWorkbench_LIBRARIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} reduce_match)

# overlap_check_match
# - An all in one executable for processing matches fresh

# extract clementine gcps
# - Executable that registers images to clementine and the provides a gcp
add_executable( extract_clementine_gcp extract_clementine_gcp.cc )
target_link_libraries( extract_clementine_gcp
                       ${Boost_LIBRARIES}
                       ${VisionWorkbench_LIBRARIES}
                       ${LAPACK_LIBRARIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} extract_clementine_gcp)

# apollo to pinhole
# - Converts an Apollo image to pinhole file
if (IS_DIRECTORY ${StereoPipeline_INCLUDE_DIR})
  add_executable( apollo_to_pinhole apollo_to_pinhole.cc )
  target_link_libraries( apollo_to_pinhole
    ${Boost_LIBRARIES}
    ${VisionWorkbench_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${StereoPipeline_LIBRARIES}
    ${ISIS_LIBRARIES}  ${CSPICE_LIBRARIES}
    ${QT_LIBRARIES})
  set(PROGRAM_LISTING ${PROGRAM_LISTING} apollo_to_pinhole)
endif()

# Setup Installation
message("Procesing:")
foreach( PROGRAM ${PROGRAM_LISTING} )

  message(${PROGRAM})
  install( TARGETS ${PROGRAM}
    DESTINATION bin )

endforeach( PROGRAM )