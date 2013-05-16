# Copyright 2010, Olivier Stasse, JRL, CNRS/AIST
#
# This file is part of sot-hrprtc-hrp2
IF(GRX_PREFIX)
  IF (${ROBOT} STREQUAL HRP2LAAS)
    CONFIGURE_FILE (grx/sotinfo-14.py.in grx/sotinfo.py)
  ELSE ()
    CONFIGURE_FILE (grx/sotinfo-10.py.in grx/sotinfo.py)
  ENDIF ()
  SET(SCRIPT_FILES ${CMAKE_CURRENT_SOURCE_DIR}/grx/guisot.py
    ${CMAKE_CURRENT_SOURCE_DIR}/grx/guiinfosot.py
    ${CMAKE_CURRENT_SOURCE_DIR}/grx/sot.py
    ${CMAKE_CURRENT_BINARY_DIR}/grx/sotinfo.py)
  INSTALL(FILES ${SCRIPT_FILES}
    DESTINATION ${ROBOT_PATH}/script
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_READ WORLD_EXECUTE)
ENDIF(GRX_PREFIX)
