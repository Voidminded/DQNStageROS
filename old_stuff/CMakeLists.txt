cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
pkg_search_module( STG REQUIRED stage )

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${OpenCV_LIBS})
    message(STATUS "OpenCV include dir: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV CXXFLAGS: ${OpenCV_CXXFLAGS_OTHER}")
    message(STATUS  ${OpenCV_LIBS})
endif(OpenCV_FOUND)

ADD_LIBRARY( dqn MODULE dqn.cc )

# expand annoying colon-separated lists into usable strings

foreach(f ${STG_CFLAGS})
     set (STG_CFLAGS_STR "${STG_CFLAGS_STR} ${f}")
 endforeach(f)

foreach(f ${STG_LDFLAGS})
     set (STG_LDFLAGS_STR "${STG_LDFLAGS_STR} ${f}")
endforeach(f)

set_target_properties( dqn PROPERTIES
 COMPILE_FLAGS "${STG_CFLAGS_STR}"
)

target_link_libraries( dqn ${LIBS})

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( dqn PROPERTIES PREFIX "" )
