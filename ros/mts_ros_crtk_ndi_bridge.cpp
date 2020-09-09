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
#include "mts_ros_crtk_ndi_bridge.h"

CMN_IMPLEMENT_SERVICES(mts_ros_crtk_ndi_bridge);


void mts_ros_crtk_ndi_bridge::bridge(const std::string & _component_name,
                                     const std::string & _interface_name,
                                     const std::string & _ros_namespace,
                                     const double _publish_period_in_seconds,
                                     const double _tf_period_in_seconds)
{
    // create factory to bridge tool as they get created
    this->add_factory_source(_component_name,
                             _interface_name,
                             _publish_period_in_seconds,
                             _tf_period_in_seconds);

    // controller specific topics, some might be CRTK compliant
    this->bridge_interface_provided(_component_name,
                                    _interface_name,
                                    _ros_namespace,
                                    _publish_period_in_seconds,
                                    _tf_period_in_seconds);
    
    /*
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
    */
}
