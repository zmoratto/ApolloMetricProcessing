# Installing python smart picker scripts
install( FILES FittingFunction.py InterestIO.py
  PERMISSIONS GROUP_READ OWNER_READ
  DESTINATION libexec )
message( "\tsmart_picker.py" )
install( FILES smart_picker.py
  PERMISSIONS GROUP_EXECUTE OWNER_EXECUTE OWNER_READ GROUP_READ
  DESTINATION bin )

# Installing supporting executables
include_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)

set(VW_DEPENDENCIES ${Boost_LIBRARIES} ${VisionWorkbench_LIBRARIES} ${LAPACK_LIBRARIES})

# IP Guided Match
# - Slightly smarter match that avoids impossible RANSAC fits.
add_executable( ip_guided_match ip_guided_match.cc )
target_link_libraries( ip_guided_match
  ${VW_DEPENDENCIES}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
set(SUPPORT_LISTING ${SUPPORT_LISTING} ip_guided_match)

# Homography_fit.cc
add_executable( homography_fit homography_fit.cc )
target_link_libraries( homography_fit
  ${VW_DEPENDENCIES} )
set(SUPPORT_LISTING ${SUPPORT_LISTING} homography_fit)

# Kriging Guided Match
add_executable( kriging_guided_match kriging_guided_match.cc )
target_link_libraries( kriging_guided_match
  ${VW_DEPENDENCIES}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
set(SUPPORT_LISTING ${SUPPORT_LISTING} kriging_guided_match)

if ( IS_DIRECTORY ${StereoPipeline_INCLUDE_DIR})
  if ( EXISTS ${QT_QMAKE_EXECUTABLE} )
    # BA filter
    add_executable( ba_filter ba_filter.cc )
    target_link_libraries( ba_filter
      ${VW_DEPENDENCIES}
      ${StereoPipeline_LIBRARIES}
      ${ISIS_LIBRARIES} ${CSPICE_LIBRARIES} ${QT_LIBRARIES} )
    set(SUPPORT_LISTING ${SUPPORT_LISTING} ba_filter)

    # Predict Homography
    add_executable( predict_homography predict_homography.cc )
    target_link_libraries( predict_homography
      ${VW_DEPENDENCIES}
      ${StereoPipeline_LIBRARIES}
      ${ISIS_LIBRARIES} ${CSPICE_LIBRARIES} ${QT_LIBRARIES} )
    set(SUPPORT_LISTING ${SUPPORT_LISTING} predict_homography)

  endif()
endif()

message("Installing  to LIBEXEC:")
foreach( SUPPORT ${SUPPORT_LISTING} )

  message("\t" ${SUPPORT})
  install( TARGETS ${SUPPORT}
    DESTINATION libexec )

endforeach( SUPPORT )