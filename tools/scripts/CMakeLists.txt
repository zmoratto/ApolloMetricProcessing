# Installing python smart picker scripts
install( FILES FittingFunction.py InterestIO.py
  PERMISSIONS GROUP_READ OWNER_READ
  DESTINATION libexec )
install( FILES smart_picker.py
  PERMISSIONS GROUP_EXECUTE OWNER_EXECUTE OWNER_READ GROUP_READ
  DESTINATION bin )

# Installing supporting executables
include_directories(${ApolloMetricProcessing_SOURCE_DIR}/src)
include_directories(${VISIONWORKBENCH_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

set(VISIONWORKBENCH_BASE_LIBRARIES
  ${Boost_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${VISIONWORKBENCH_CORE_LIBRARY}
  ${VISIONWORKBENCH_MATH_LIBRARY}
  ${VISIONWORKBENCH_IMAGE_LIBRARY}
  ${VISIONWORKBENCH_FILEIO_LIBRARY}
  )

# IP Guided Match
# - Slightly smarter match that avoids impossible RANSAC fits.
#add_executable( ip_guided_match ip_guided_match.cc )
#target_link_libraries( ip_guided_match
#  ${VW_DEPENDENCIES}
#  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
#set(SUPPORT_LISTING ${SUPPORT_LISTING} ip_guided_match)

# Kriging Guided Match
#add_executable( kriging_guided_match kriging_guided_match.cc )
#target_link_libraries( kriging_guided_match
#  ${VW_DEPENDENCIES}
#  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib/libANN.a )
#set(SUPPORT_LISTING ${SUPPORT_LISTING} kriging_guided_match)

set(APOLLO_USED_LIBS
  ${VISIONWORKBENCH_BASE_LIBRARIES}
  ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
  )

# Homography_fit.cc
add_apollo_hidden( homography_fit homography_fit.cc )

if (StereoPipeline_FOUND AND QT_FOUND)
  include_directories(${StereoPipeline_INCLUDE_DIRS})
  include_directories(${ISIS_INCLUDE_DIRS})
  include_directories(${CSPICE_INCLUDE_DIRS})

  set(APOLLO_USED_LIBS
    ${VISIONWORKBENCH_BASE_LIBRARIES}
    ${VISIONWORKBENCH_INTERESTPOINT_LIBRARY}
    ${VISIONWORKBENCH_BUNDLEADJUSTMENT_LIBRARY}
    ${VISIONWORKBENCH_STEREO_LIBRARY}
    ${StereoPipeline_LIBRARIES}
    ${ISIS_LIBRARIES} ${CSPICE_LIBRARIES} ${QT_LIBRARIES}
    )

  add_apollo_hidden( ba_filter ba_filter.cc )

  set(APOLLO_USED_LIBS
    ${APOLLO_USED_LIBS}
    ${VISIONWORKBENCH_CARTOGRAPHY_LIBRARY}
    )

  add_apollo_hidden( predict_homography predict_homography.cc )

endif()