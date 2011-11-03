project(ApolloMetricProcessing CXX C)
cmake_minimum_required(VERSION 2.8)

# Add path for custom modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  "${ApolloMetricProcessing_SOURCE_DIR}/CMakeModules"
  )

set(PACKAGE_VERSION "0.1")
set(PACKAGE_NAME ApolloMetricProcessing)
set(PACKAGE_STRING "${PACKAGE_NAME} ${PACKAGE_VERSION}")
set(PACKAGE_BUGREPORT "z.m.moratto@nasa.gov")

# Custom ApolloMetricProcessing options
option(ENABLE_RPATHS
   "Use RPaths in executables to find shared libraries." FALSE)
if (ENABLE_RPATHS)
  set(CMAKE_SKIP_RPATH                  FALSE CACHE BOOL "")
  set(CMAKE_SKIP_BUILD_RPATH            FALSE CACHE BOOL "")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH    FALSE CACHE BOOL "")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE  CACHE BOOL "")
  if(APPLE)
    set(CMAKE_INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib)
  endif(APPLE)
  mark_as_advanced(
    CMAKE_SKIP_RPATH
    CMAKE_SKIP_BUILD_RPATH
    CMAKE_BUILD_WITH_INSTALL_RPATH
    CMAKE_INSTALL_RPATH
    CMAKE_INSTALL_RPATH_USE_LINK_PATH
    )
endif(ENABLE_RPATHS)

# Defining packages
include(AddApollo)
include(CheckIncludeFiles)
include(CheckFunctionExists)
find_package(Boost REQUIRED
  COMPONENTS program_options system thread filesystem iostreams)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -DBOOST_FILESYSTEM_VERSION=2)
find_package(VisionWorkbench REQUIRED
  COMPONENTS core image math fileio camera interestpoint cartography bundleadjustment stereo)
find_package(LAPACK-eigen)
find_package(StereoPipeline
  COMPONENTS core isisio)
find_package(ISIS)
find_package(CSPICE)
find_package(Qt4)
find_package(FLANN)
find_package(OpenSceneGraph
  COMPONENTS osgViewer osgGA)

# Check for important features in the 3rd party libraries that we use
# in this project.
check_function_exists(boost::iostreams::gzip_compressor, HAVE_BOOST_IOSTREAMS_GZIP)
check_include_files(boost/polygon/polygon.hpp HAVE_BOOST_POLYGON_H)

# Blanket including of QT, move this after we work out which packages
# truly need this.
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
endif()

# Subdirectories
add_subdirectory(src)
add_subdirectory(scripts)
add_subdirectory(pba)