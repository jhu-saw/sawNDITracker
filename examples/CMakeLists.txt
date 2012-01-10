#
# $Id$
#
# (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# create a list of libraries needed for this project
set (REQUIRED_CISST_LIBRARIES cisstCommon cisstCommonQt cisstCommonXML
                              cisstVector cisstOSAbstraction cisstNumerical
                              cisstMultiTask cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawNDITracker has been compiled within cisst, we should find it automatically
  find_package (sawNDITracker REQUIRED HINTS "${cisst_saw_BINARY_DIR}/saw/components")

  if (sawNDITracker_FOUND)

    # sawNDITracker configuration
    include_directories (${sawNDITracker_INCLUDE_DIR})
    link_directories (${sawNDITracker_LIBRARY_DIR})

    add_executable (sawNDITrackerQtExample mainQt.cpp)
    set_property (TARGET sawNDITrackerQtExample PROPERTY FOLDER "sawNDITracker")

    # link against non cisst libraries and saw components
    target_link_libraries (sawNDITrackerQtExample
                           ${sawNDITracker_LIBRARIES}
                           ${sawNDITrackerQt_LIBRARIES}
                           ${QT_LIBRARIES}
                         )
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawNDITrackerQtExample ${REQUIRED_CISST_LIBRARIES})

  endif (sawNDITracker_FOUND)

endif (cisst_FOUND)
