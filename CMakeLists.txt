# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of FootStepPlannerRTC.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/idlrtc.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME hrprtc-hrp2-rtc)
SET(PROJECT_DESCRIPTION "RTC HRP-2 Real-Time Component.")
SET(PROJECT_URL "https://github.com/stack-of-tasks/sot-hrprtc-hrp2.git")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)

SETUP_PROJECT()

# Remove pedantic flag as Corba stubs won't compile with it.
STRING(REPLACE "-pedantic" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.1")
ADD_REQUIRED_DEPENDENCY("openrtm-aist")
ADD_REQUIRED_DEPENDENCY("sot-core")

# Define additional options.
FIND_PATH(GRX_PREFIX
  # Make sure bin/DynamicsSimulator exists (to validate that it is
  # _really_ the prefix of an OpenHRP setup).
  include/rtm/idl/SDOPackage.hh
  HINTS /opt/grx/
  DOC "GRX software prefix (i.e. '/opt/grxX.Y')"
  NO_DEFAULT_PATH)

IF(!GRX_PREFIX)
  MESSAGE(ERROR "Dit not find GRX_PREFIX")
ENDIF(!GRX_PREFIX)

SET(ROBOT "" CACHE STRING "GRX robot name (HRP2JRL or HRP2LAAS)")
SET(ROBOT_PATH "${GRX_PREFIX}/${ROBOT}")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(components)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
