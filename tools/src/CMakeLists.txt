# Make sure linker and compiler can find their way
include_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
link_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
message(STATUS "Adding ${ApolloMetricProcessing_SOURCE_DIR}/src")

set(VW_DEPENDENCIES ${Boost_LIBRARIES} ${VisionWorkbench_LIBRARIES} ${LAPACK_LIBRARIES})

# overlap_check_match
# - An all in one executable for processing matches fresh

# extract clementine gcps
# - Executable that registers images to clementine and the provides a gcp
add_executable( extract_clementine_gcp extract_clementine_gcp.cc )
target_link_libraries( extract_clementine_gcp
                       ${VW_DEPENDENCIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} extract_clementine_gcp)

# VWIP Filter
# - Remove IPs that are close to reseaus, this is specifically for Apollo
add_executable( vwip_filter vwip_filter.cc )
target_link_libraries( vwip_filter
                       ${VW_DEPENDENCIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} vwip_filter)

# Apollo Match
# - Slightly smarter match that avoids impossible RANSAC fits.
add_executable( apollo_match apollo_match.cc )
target_link_libraries( apollo_match
  ${VW_DEPENDENCIES}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
set(PROGRAM_LISTING ${PROGRAM_LISTING} apollo_match)

# Apollo Bulk Match
add_executable( apollo_bulk_match apollo_bulk_match.cc )
target_link_libraries( apollo_bulk_match
  ${VW_DEPENDENCIES}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
set(PROGRAM_LISTING ${PROGRAM_LISTING} apollo_bulk_match )

# Bulk Match Unpack
add_executable( bulk_match_unpack bulk_match_unpack.cc )
target_link_libraries( bulk_match_unpack
  ${VW_DEPENDENCIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} bulk_match_unpack )

# IP Disparity
add_executable( ip_disparity ip_disparity.cc )
target_link_libraries( ip_disparity
  ${VW_DEPENDENCIES} )
set(PROGRAM_LISTING ${PROGRAM_LISTING} ip_disparity )

if (IS_DIRECTORY ${StereoPipeline_INCLUDE_DIR})

  # apollo to pinhole
  # - Converts an Apollo image to pinhole file
  add_executable( apollo_to_pinhole apollo_to_pinhole.cc )
  target_link_libraries( apollo_to_pinhole
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES}
    ${ISIS_LIBRARIES}  ${CSPICE_LIBRARIES}
    ${QT_LIBRARIES})
  set(PROGRAM_LISTING ${PROGRAM_LISTING} apollo_to_pinhole)

  # GCP from Control Network
  # - Converts an ISIS style to individual GCPs
  add_executable( gcp_from_net gcp_from_net.cc )
  target_link_libraries( gcp_from_net
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES}
    ${ISIS_LIBRARIES}  ${CSPICE_LIBRARIES}
    ${QT_LIBRARIES})
  set(PROGRAM_LISTING ${PROGRAM_LISTING} gcp_from_net)

  # CNET USGS Conversion
  add_executable( cnet_usgs_conv cnet_usgs_conv.cc )
  target_link_libraries( cnet_usgs_conv
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES} )
  set(PROGRAM_LISTING ${PROGRAM_LISTING} cnet_usgs_conv)

  # CSV 2 ISIS ADJUST
  add_executable( csv_2_isis_adjust csv_2_isis_adjust.cc )
  target_link_libraries( csv_2_isis_adjust
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES} )
  set(PROGRAM_LISTING ${PROGRAM_LISTING} csv_2_isis_adjust)

  # ISIS ADJUST 2 CSV
  add_executable( isis_adjust_2_csv isis_adjust_2_csv.cc )
  target_link_libraries( isis_adjust_2_csv
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES} )
  set(PROGRAM_LISTING ${PROGRAM_LISTING} isis_adjust_2_csv)

  # Apollo Guided Match
  add_executable( apollo_guided_match apollo_guided_match.cc )
  target_link_libraries( apollo_guided_match
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES}
    ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
  set(PROGRAM_LISTING ${PROGRAM_LISTING} apollo_guided_match)

  # Extract LOLA and WAC GCP
  add_executable( extract_lola_wac_gcp extract_lola_wac_gcp.cc )
  target_link_libraries( extract_lola_wac_gcp
    ${VW_DEPENDENCIES}
    ${StereoPipeline_LIBRARIES} )
  set(PROGRAM_LISTING ${PROGRAM_LISTING} extract_lola_wac_gcp)
endif()

message("Installing to BIN:")
foreach( PROGRAM ${PROGRAM_LISTING} )

  message( "\t" ${PROGRAM})
  install( TARGETS ${PROGRAM}
    DESTINATION bin )

endforeach( PROGRAM )