#
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.6.2)

project (sawNDITracker)

# find cisst to obtain configuration flags
find_package (cisst REQUIRED cisstCommon)

add_subdirectory (code)
if(CISST_HAS_QT)
  add_subdirectory (examples)
endif()
