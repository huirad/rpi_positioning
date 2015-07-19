###########################################################################
# @licence app begin@
# SPDX-License-Identifier: MPL-2.0
#
# Component Name: Logger
#
# Author: Helmut Schmidt
# 
# License:
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with
# this file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Update (2015/07/10) : Helmut Schmidt <https://github.com/huirad>,
#		       - first version derived from log-replayer CMakeLists.txt
#
# @licence end@
###########################################################################


message(STATUS "TEST-LOGGER")
message(STATUS "WITH_DLT = ${WITH_DLT}")
message(STATUS "WITH_GPSD = ${WITH_GPSD}")
message(STATUS "WITH_NMEA = ${WITH_NMEA}")
message(STATUS "WITH_REPLAYER = ${WITH_REPLAYER}")
message(STATUS "WITH_DEBUG = ${WITH_DEBUG}")

include_directories("${PROJECT_SOURCE_DIR}/inc")

find_package(PkgConfig)

set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/test-poslog.cpp)
add_executable(test-poslog ${SRCS})
set(LIBRARIES pthread poslog)

if(WITH_DLT)
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")
    find_package(DLT REQUIRED)
    add_definitions("-DDLT_ENABLED=1")
    include_directories( ${DLT_INCLUDE_DIRS} )
    set(LIBRARIES ${LIBRARIES} ${DLT_LIBRARIES})
endif()

if(WITH_DEBUG)
    add_definitions("-DDEBUG_ENABLED=1")
endif()

target_link_libraries(test-poslog ${LIBRARIES})

install(TARGETS test-poslog DESTINATION bin)


set(SRCS ${CMAKE_CURRENT_SOURCE_DIR}/test-gnsslog.cpp
${CMAKE_CURRENT_SOURCE_DIR}/gnsslog.cpp)
add_executable(test-gnsslog ${SRCS})
set(LIBRARIES pthread poslog)


set(gnss-service_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../gnss-service/api")
set(gnss-service_LIBRARY_DIRS "${PROJECT_BINARY_DIR}/../gnss-service/src")
if(WITH_GPSD)
    set(GNSS_LIBRARIES "gnss-service-use-gpsd")
elseif(WITH_NMEA)
    set(GNSS_LIBRARIES "gnss-service-use-nmea")
elseif(WITH_REPLAYER)
    set(GNSS_LIBRARIES "gnss-service-use-replayer")
else()
    message(STATUS "Invalid cmake options!")
endif()
message(STATUS "GNSS_LIBRARIES = ${GNSS_LIBRARIES}")
set(LIBRARIES ${LIBRARIES} ${GNSS_LIBRARIES})

include_directories(${gnss-service_INCLUDE_DIRS})

if(WITH_DLT)
    set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")
    find_package(DLT REQUIRED)
    add_definitions("-DDLT_ENABLED=1")
    include_directories( ${DLT_INCLUDE_DIRS} )
    set(LIBRARIES ${LIBRARIES} ${DLT_LIBRARIES})
endif()

if(WITH_DEBUG)
    add_definitions("-DDEBUG_ENABLED=1")
endif()

target_link_libraries(test-gnsslog ${LIBRARIES})
#message(STATUS "LIBRARIES = ${LIBRARIES}")

install(TARGETS test-gnsslog DESTINATION bin)

