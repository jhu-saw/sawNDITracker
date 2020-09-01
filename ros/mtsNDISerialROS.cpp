/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-11-28

  (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include "mtsNDISerialROS.h"

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsNDISerialROS, mtsTaskFromSignal, mtsTaskContinuousConstructorArg);

mtsNDISerialROS::mtsNDISerialROS(const std::string & componentName):
    mtsTaskFromSignal(componentName)
{
    Init();
}

mtsNDISerialROS::mtsNDISerialROS(const mtsTaskContinuousConstructorArg & argument):
    mtsTaskFromSignal(argument.Name)
{
    Init();
}
void mtsNDISerialROS::Init(void)
{
    mROSBridge = 0;
    mTFBridge = 0;

    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("ToolNames", Tracker.ToolNames);
        interfaceRequired->AddEventHandlerWrite(&mtsNDISerialROS::ConnectedEventHandler,
                                                this, "Connected");
        interfaceRequired->AddEventHandlerVoid(&mtsNDISerialROS::UpdatedToolsEventHandler,
                                               this, "UpdatedTools");
    }
}

void mtsNDISerialROS::AddROSTopics(const std::string & rosBridgeName,
                                   const std::string & tfBridgeName,
                                   const std::string & trackerName)
{
    mROSBridgeName = rosBridgeName;
    mTFBridgeName = tfBridgeName;
    mTrackerName = trackerName;

    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    // get bridge for topics
    mtsComponent * component = manager->GetComponent(mROSBridgeName);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to find any component with name \""
                                 << mROSBridgeName << "\"" << std::endl;
        return;
    }

    mROSBridge = dynamic_cast<mtsROSBridge *>(component);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: component with name \""
                                 << mROSBridgeName
                                 << "\" doesn't seem to be of type \"mtsROSBridge\""
                                 << std::endl;
        return;
    }

    // get bridge for tf2
    component = manager->GetComponent(mTFBridgeName);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to find any component with name \""
                                 << mTFBridgeName << "\"" << std::endl;
        return;
    }

    mTFBridge = dynamic_cast<mtsROSBridge *>(component);
    if (!component) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: component with name \""
                                 << mTFBridgeName
                                 << "\" doesn't seem to be of type \"mtsROSBridge\""
                                 << std::endl;
        return;
    }

    // add some controller ROS topics
    // connection
    mROSBridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Controller", "Connect", "connect");
    mROSBridge->AddSubscriberToCommandVoid
        ("Controller", "Disconnect", "disconnect");
    mROSBridge->AddPublisherFromEventWrite<std::string, std_msgs::String>
        ("Controller", "Connected", "connected");
    // beep
    mROSBridge->AddSubscriberToCommandWrite<int, std_msgs::Int32>
        ("Controller", "Beep", "beep");
    // tracking
    mROSBridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        ("Controller", "ToggleTracking", "track");
    mROSBridge->AddPublisherFromEventWrite<bool, std_msgs::Bool>
        ("Controller", "Tracking", "tracking");

    // stray markers
    mROSBridge->AddPublisherFromCommandRead<std::vector<vct3>, sensor_msgs::PointCloud>
        ("Controller", "MarkerPositions",
         "fiducials");
    mROSBridge->AddPublisherFromCommandRead<std::vector<vct3>, sensor_msgs::PointCloud>
        ("Controller", "MarkerPositionsLocal",
         "fiducials_local");

    // errors
    mROSBridge->AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        ("Controller", "Error",
         "error");

    // connect bridge to NDI component::Controller interface
    manager->Connect(mROSBridgeName, "Controller",
                     mTrackerName, "Controller");
}

void mtsNDISerialROS::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsNDISerialROS::Run(void)
{
    ProcessQueuedEvents();
}

void mtsNDISerialROS::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsNDISerialROS::ConnectedEventHandler(const std::string & connected)
{
    if (connected != "") {
        UpdatedToolsEventHandler();
    }
}

void mtsNDISerialROS::UpdatedToolsEventHandler(void)
{
    if (!mROSBridge) {
        CMN_LOG_CLASS_INIT_ERROR << "UpdatedToolsEventHandler: null pointer for mROSBridge.  This pointer should have been initialized in method AddROSTopics" << std::endl;
        return;
    }

    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    std::vector<std::string> toolNames;
    Tracker.ToolNames(toolNames);
    for (size_t i = 0; i < toolNames.size(); ++i) {
        std::string name = toolNames[i];
        std::string rosName = name;
        std::replace(rosName.begin(), rosName.end(), '-', '_');
        if (isdigit(rosName[0])) {
            rosName = "ndi_" + rosName;
        }
        // check if there's already a required interface with that name
        if (!(mROSBridge->GetInterfaceRequired(name))) {
            // publishers
            mROSBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
                (name, "measured_cp", rosName + "/measured_cp");
            mROSBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
                (name, "measured_cp_local", rosName + "/local/measured_cp");
            manager->Connect(mROSBridgeName, name,
                             mTrackerName, name);
            // tf2
            mTFBridge->Addtf2BroadcasterFromCommandRead(name, "measured_cp");
            manager->Connect(mTFBridgeName, name,
                             mTrackerName, name);
        }
    }
}
