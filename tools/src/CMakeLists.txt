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
  )

add_apollo_tool( extract_clementine_gcp extract_clementine_gcp.cc )
add_apollo_tool( apollo_match apollo_match.cc )
add_apollo_tool( apollo_bulk_match apollo_bulk_match.cc )

set(APOLLO_USED_LIBS
  ${VISIONWORKBENCH_BASE_LIBRARIES}
  ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
  )

add_apollo_tool( vwip_filter vwip_filter.cc )
add_apollo_tool( bulk_match_unpack bulk_match_unpack.cc )
add_apollo_tool( ip_disparity ip_disparity.cc )

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
    )

  add_apollo_tool( apollo_to_pinhole apollo_to_pinhole.cc )

  set(APOLLO_USED_LIBS ${APOLLO_USED_LIBS}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
    )

  add_apollo_tool( gcp_from_net gcp_from_net.cc )
  add_apollo_tool( lola_relookup lola_relookup.cc )

  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${StereoPipeline_LIBRARIES}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    )

  add_apollo_tool( cnet_usgs_conv cnet_usgs_conv.cc )
  add_apollo_tool( csv_2_isis_adjust csv_2_isis_adjust.cc )
  add_apollo_tool( isis_adjust_2_csv isis_adjust_2_csv.cc )

  set(APOLLO_USED_LIBS
    ${APOLLO_USED_LIBS}
    ${VISIONWORKBENCH_STEREO_LIBRARY}
    ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
    ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
    )

  add_apollo_tool( refine_ip refine_ip.cc )

  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${StereoPipeline_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
    ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
    )

  add_apollo_tool( extract_lola_wac_gcp extract_lola_wac_gcp.cc )
  add_apollo_tool( extract_lola_wac_gcp2 extract_lola_wac_gcp2.cc )
  add_apollo_tool( extract_lola_wac_gcp3 extract_lola_wac_gcp3.cc )
  add_apollo_tool( solve_apollo_gcp solve_apollo_gcp.cc )

endif()

if (OPENSCENEGRAPH_FOUND)

  include_directories( ${OPENSCENEGRAPH_INCLUDE_DIRS} )
  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${OPENSCENEGRAPH_LIBRARIES}
    )
  add_apollo_tool( pc_display pc_display.cc )

endif()
