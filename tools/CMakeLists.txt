message("Configuration Started:")
message("--------------------------------------------")
project(ApolloMetricProcessing CXX C)
cmake_minimum_required(VERSION 2.6)
set(CMAKE_MODULE_PATH "${ApolloMetricProcessing_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

# Setting up RPATH
set( CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" )

if(APPLE)
  set( CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib )
endif(APPLE)

set( CMAKE_SKIP_RPATH                   FALSE CACHE BOOL "" )
set( CMAKE_SKIP_BUILD_RPATH             FALSE CACHE BOOL "" )
set( CMAKE_BUILD_WITH_INSTALL_RPATH     FALSE CACHE BOOL "" )
set( CMAKE_INSTALL_RPATH_USE_LINK_PATH  TRUE  CACHE BOOL "" )

mark_as_advanced(
  CMAKE_INSTALL_RPATH
  CMAKE_SKIP_RPATH
  CMAKE_SKIP_BUILD_RPATH
  CMAKE_BUILD_WITH_INSTALL_RPATH
  CMAKE_INSTALL_RPATH_USE_LINK_PATH
)

# Building Dep
if(APPLE)
  execute_process( COMMAND make macosx-g++
    WORKING_DIRECTORY ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2 )
else(APPLE)
  execute_process( COMMAND make linux-g++
    WORKING_DIRECTORY ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2 )
endif(APPLE)

# Defining packages
find_package(Boost REQUIRED
  COMPONENTS program_options system thread filesystem iostreams)
find_package(VisionWorkbench REQUIRED
  COMPONENTS core image math fileio camera interestpoint cartography bundleadjustment stereo)
find_package(LAPACK-eigen)
find_package(StereoPipeline
  COMPONENTS core isisio)
find_package(ISIS)
find_package(CSPICE)
find_package(QT)
find_package(FLANN)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${VisionWorkbench_INCLUDE_DIR}
  ${LAPACK_INCLUDE_DIR}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/include)
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${VisionWorkbench_LIBRARY_DIRS}
  ${LAPACK_LIBRARIES_DIR}
  ${ApolloMetricProcessing_SOURCE_DIR}/thirdparty/ann_1.1.2/lib)

if (IS_DIRECTORY ${StereoPipeline_INCLUDE_DIR})
  include_directories(${StereoPipeline_INCLUDE_DIR})
  link_directories(${StereoPipeline_LIBRARY_DIRS})
endif()

if (IS_DIRECTORY ${ISIS_INCLUDE_DIR})
  include_directories(${ISIS_INCLUDE_DIR})
  link_directories(${ISIS_LIBRARY_DIRS})
endif()

if (CSPICE_FOUND)
  include_directories(${CSPICE_INCLUDE_DIR})
  link_directories(${CSPICE_LIBRARY_DIRS})
endif()

if (FLANN_FOUND)
  include_directories(${FLANN_INCLUDE_DIR})
  link_directories(${FLANN_LIBRARY_DIRS})
endif()

if (QT_FOUND)
  set(QT_MODS "Qt" "Qt3Support" "QtAssistant" "QtCore" "QtDBus" "QtDesigner" "QtGui" "QtHelp" "QtMultimedia" "QtNetwork" "QtOpenGL" "QtScript" "QtScriptTools" "QtSql" "QtSvg" "QtTest" "QtUiTools" "QtWebKit" "QtXml" "QtXmlPatterns")
  foreach( QTBASE ${QT_INCLUDE_DIR} )
    foreach( QTMOD ${QT_MODS} )
      if( IS_DIRECTORY ${QTBASE}/${QTMOD} )
        include_directories(${QTBASE}/${QTMOD})
      endif()
    endforeach()
  endforeach()
  include_directories(${QT_INCLUDE_DIR})
  link_directories(${QT_LIBRARY_DIRS})
endif()

# Subdirectories
add_subdirectory(src)
add_subdirectory(scripts)

set(CMAKE_BUILD_TYPE Release)