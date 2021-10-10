#
# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawNDITrackerShare VERSION 1.1.0)

install (DIRECTORY
         "${sawNDITrackerShare_SOURCE_DIR}"
         DESTINATION share/sawNDITracker
         COMPONENT sawNDITracker-Share
         PATTERN "CMakeLists.txt" EXCLUDE)
