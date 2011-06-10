macro(add_apollo_executable name)
  add_executable(${name} ${ARGN})
  foreach(lib ${APOLLO_USED_LIBS})
    if( NOT ${lib} STREQUAL "optimized" AND
        NOT ${lib} STREQUAL "debug" )
    	target_link_libraries( ${name} ${lib} )
    endif()
  endforeach(lib)
endmacro(add_apollo_executable name)

macro(add_apollo_tool name)
  message( "ARGN: " ${ARGN} )
  add_apollo_executable(${name} ${ARGN})
  message( "Added: " ${name} )
  install(TARGETS ${name} RUNTIME DESTINATION bin)
endmacro(add_apollo_tool name)

macro(add_apollo_hidden name)
  add_apollo_executable(${name} ${ARGN})
  message( "Added: " ${name} )
  install(TARGETS ${name} RUNTIME DESTINATION libexec)
endmacro(add_apollo_hidden name)

function(apollo_enable_testing)
  set(GTEST_DIR ${CMAKE_SOURCE_DIR}/thirdparty/gtest)
  include_directories(${VISIONWORKBENCH_INCLUDE_DIRS})
  include_directories(${GTEST_DIR}/include)
  add_library(gtest SHARED EXCLUDE_FROM_ALL
    ${GTEST_DIR}/src/gtest-all.cc
    ${CMAKE_SOURCE_DIR}/src/test/test_main.cc
    )
  target_link_libraries(gtest
    ${VISIONWORKBENCH_CORE_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    )
  set_target_properties(gtest
    PROPERTIES
    COMPILE_FLAGS "-DTEST_SRCDIR=\\\"${CMAKE_CURRENT_SOURCE_DIR}\\\" -DTEST_DSTDIR=\\\"${CMAKE_CURRENT_BINARY_DIR}\\\""
    )

  add_custom_target(check)
  add_dependencies(check gtest)
endfunction(apollo_enable_testing)


function(apollo_add_test source_file)
  string(REGEX REPLACE "^([A-Za-z0-9_]*)\\.([A-Za-z0-9]*)" "\\1" executable "${source_file}")

  add_executable(${executable} EXCLUDE_FROM_ALL ${source_file} )
  target_link_libraries( ${executable} gtest )
  foreach(lib ${APOLLO_USED_LIBS})
    target_link_libraries( ${executable} ${lib} )
  endforeach(lib)

  set_target_properties(${executable}
    PROPERTIES
    COMPILE_FLAGS "-DTEST_SRCDIR=\\\"${CMAKE_CURRENT_SOURCE_DIR}\\\" -DTEST_DSTDIR=\\\"${CMAKE_CURRENT_BINARY_DIR}\\\""
    )
  add_custom_target(${executable}Exec ${CMAKE_CURRENT_BINARY_DIR}/${executable}
    DEPENDS ${executable}
    )
  add_dependencies(check ${executable}Exec)
endfunction(apollo_add_test)