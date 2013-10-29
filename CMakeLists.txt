# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of FootStepPlannerRTC.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/idlrtc.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/openhrp.cmake)

SET(PROJECT_NAME hrprtc-hrp2-rtc)
SET(PROJECT_DESCRIPTION "RTC HRP Real-Time Component.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-hrprtc-hrp2.git")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

# Remove pedantic flag as Corba stubs won't compile with it.
STRING(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.1")
ADD_REQUIRED_DEPENDENCY("openrtm-aist")
ADD_REQUIRED_DEPENDENCY("sot-core")

SEARCH_GRX()
SEARCH_GRX_ROBOTS("HRP2JRL;HRP2LAAS;HRP4LIRMM")

SET(STABILIZER ON CACHE BOOL "Whether stabilizer RTcomponent is used or not.")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(components)
ADD_SUBDIRECTORY(scripts)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
