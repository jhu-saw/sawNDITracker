/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-11-28

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsNDISerialROS_h
#define _mtsNDISerialROS_h

class mtsROSBridge;

#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>

class mtsNDISerialROS: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsNDISerialROS(const std::string & componentName);
    mtsNDISerialROS(const mtsTaskContinuousConstructorArg & argument);

    inline ~mtsNDISerialROS() {}

    inline void Configure(const std::string & CMN_UNUSED(filename)) {};
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void AddROSTopics(const std::string & rosBridgeName,
                      const std::string & tfBridgeName,
                      const std::string & trackerName,
                      const std::string & rosNamespace);
 private:
    void Init(void);

    struct {
        mtsFunctionRead ToolNames;
    } Tracker;

    std::string mROSBridgeName;
    std::string mTFBridgeName;
    std::string mTrackerName;
    std::string mROSNamespace;
    mtsROSBridge * mROSBridge;
    mtsROSBridge * mTFBridge;

    void ConnectedEventHandler(const std::string & connected);
    void UpdatedToolsEventHandler(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNDISerialROS);

#endif // _mtsNDISerialROS_h
