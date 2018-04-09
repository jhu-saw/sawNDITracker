/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Ali Uneri
  Created on: 2009-10-13

  (C) Copyright 2009-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief SAW component for NDI surgical trackers with serial (RS-232) interface.

  \warning Missing support for 14400bps, 921600bps and 1228739bps baud rates in osaSerialPort.

  \todo Consider deriving from mtsTaskContinuous using an "adaptive" sleep.
  \todo Verify the need for existing sleep times.
  \todo Enable individual tools on-the-fly (dynamically add their interfaces and Qt Widget).
  \todo Move CalibratePivot to cisstNumerical?
  \todo Error handling for partial and concatenated messages in ResponseRead()
  \todo Handle other main types of tools (besides pointer, reference, etc.).
  \todo Parse port/system status, in order to get "partially out of volume", etc..
  \todo Refactor ComputeCRC and implement a CRC check in CommandSend (move CRC check to osaSerialPort?).
  \todo Every sscanf should check if valid number of items have been read (wrapper for sscanf?).
  \todo Error handling for all strncpy.
  \todo Check for buffer overflow in CommandAppend.
  \todo Support for the extra features of newer Polaris versions.
  \todo Pretty print for SerialNumber, to extract date, etc..
  \todo Create one state table per tool
  \todo Use frame number to decide if timestamp should be refreshed (not needed if one state table per tool)
  \todo Use a map to convert Tool's MainType to human readable text.
  \todo Strategies for error recovery, send an event with a human readable payload, implement in CheckResponse.
*/

#ifndef _mtsNDISerial_h
#define _mtsNDISerial_h

#include <cisstCommon/cmnPath.h>

#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstOSAbstraction/osaSerialPort.h>
#include <cisstOSAbstraction/osaStopwatch.h>

#include <cisstMultiTask/mtsMatrix.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <cisstParameterTypes/prmPositionCartesianGet.h>

#include <sawNDITracker/sawNDITrackerConfig.h>
#include <sawNDITracker/sawNDITrackerExport.h>  // always include last


class CISST_EXPORT mtsNDISerial : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 protected:
    class Tool
    {
     public:
        Tool(void);
        ~Tool(void) {};

        std::string Name;
        unsigned int FrameNumber;
        double ErrorRMS;

        mtsInterfaceProvided * Interface;
        prmPositionCartesianGet PositionLocal; // wrt camera
        prmPositionCartesianGet Position; // wrt reference frame
        std::string ReferenceFrame;
        Tool * ReferenceTool;

        std::string UniqueID;
        char PortHandle[3];
        // PHINF 0001
        std::string MainType;
        std::string ManufacturerID;
        std::string ToolRevision;
        std::string SerialNumber;
        // PHINF 0004
        std::string PartNumber;
        std::string Definition;
    };

 public:
    inline mtsNDISerial(const std::string & taskName, const double period):
        mtsTaskPeriodic(taskName, period, false, 5000),
        mTools("Tools", this->Services()),
        mPortToTool("PortToTool", this->Services())
    {
        Init();
    }

    inline mtsNDISerial(const mtsTaskPeriodicConstructorArg & arg):
        mtsTaskPeriodic(arg),
        mTools("Tools", this->Services()),
        mPortToTool("PortToTool", this->Services())
    {
        Init();
    }

    ~mtsNDISerial(void) {};

    /*! Set the name of the serial port to use.  If the serial port is
      defined using this method the field "port" in the
      configuration file will be ignored. */
    void SetSerialPort(const std::string & serialPort);

    /*! Configure the tracker using a JSON file.  Configure will
      create the tools provided interfaces and save the settings
      (serial number, rom... but will not check if these are valid.
      Port search and rom loading will happen when first
      connected. */
    void Configure(const std::string & filename = "");
    inline void Startup(void) {};
    void Run(void);
    void Cleanup(void);

    inline size_t GetNumberOfTools(void) const {
        return mTools.size();
    }
    std::string GetToolName(const size_t index) const;

    /*! Connect using specified port.  If the string is empty the
      method will try to use whatever port is already defined in the
      class. */
    void Connect(const std::string & serialPortName);
    void Disconnect(void);

    /*! Initialization sequence to initialize, query and enable all
      tool handles.  Calls Initialize, PortHandlesInitialize,
      PortHandlesPassiveTools, PortHandlesQuery, PortHandlesEnable. */
    void InitializeAll(void);

    /*! Send INIT to device and waits for response OKAY */
    void Initialize(void);
    void PortHandlesInitialize(void);
    void PortHandlesQuery(void);
    void PortHandlesEnable(void);
    void PortHandlesPassiveTools(void);

 protected:
    enum { MAX_BUFFER_SIZE = 512 };
    enum { CRC_SIZE = 4 };

    void Init(void);

    inline size_t GetSerialBufferSize(void) const {
        return mSerialBufferPointer - mSerialBuffer;
    }
    inline size_t GetSerialBufferAvailableSize(void) const {
        return MAX_BUFFER_SIZE - GetSerialBufferSize();
    }

    inline size_t GetSerialBufferStringSize(void) const {
        if (GetSerialBufferSize() == 0)  {
            CMN_LOG_CLASS_RUN_ERROR << "GetSerialBufferStringSize: string is empty and not null terminated" << std::endl;
            return 0;
        } else if (*(mSerialBufferPointer - 1) == '\0') {
            return GetSerialBufferSize() - 1;
        }
        CMN_LOG_CLASS_RUN_ERROR << "GetSerialBufferStringSize: string is not null terminated" << std::endl;
        return 0;
    }

    void CommandInitialize(void);
    void CommandAppend(const char command);
    void CommandAppend(const char * command);
    void CommandAppend(const int command);
    bool CommandSend(void);

    inline bool CommandSend(const char * command) {
        CommandInitialize();
        CommandAppend(command);
        return CommandSend();
    }
    bool ResponseRead(void);
    bool ResponseRead(const char * expectedMessage);
    unsigned int ComputeCRC(const char * data);
    bool ResponseCheckCRC(void);

    bool ResetSerialPort(void);
    bool SetSerialPortSettings(osaSerialPort::BaudRateType baudRate,
                               osaSerialPort::CharacterSizeType characterSize,
                               osaSerialPort::ParityCheckingType parityChecking,
                               osaSerialPort::StopBitsType stopBits,
                               osaSerialPort::FlowControlType flowControl);
    void Beep(const int & numberOfBeeps);

    void LoadToolDefinitionFile(const char * portHandle, const std::string & filePath);
    Tool * CheckTool(const std::string & uniqueID);
    Tool * AddTool(const std::string & name,
                   const std::string & uniqueID,
                   const std::string & toolDefinitionFile = "",
                   const std::string & reference = "");

    void ToggleTracking(const bool & track);
    void ToggleStrayMarkers(const bool & stray);
    void Track(void);

    struct {
        mtsFunctionWrite Connected;
        mtsFunctionWrite Tracking;
        mtsFunctionVoid UpdatedTools;
    } Events;

    mtsStateTable * mConfigurationStateTable;
    mtsInterfaceProvided * mControllerInterface;

    std::string mSerialPortName;
    osaSerialPort mSerialPort;
    char mSerialBuffer[MAX_BUFFER_SIZE];
    char * mSerialBufferPointer;

    cmnPath mDefinitionPath;

    typedef cmnNamedMap<Tool> ToolsType;
    ToolsType mTools;
    cmnNamedMap<Tool> mPortToTool;
    std::vector<std::string> mToolNames;

    bool mIsTracking;
    bool mTrackStrayMarkers;
    mtsMatrix<double> mStrayMarkers;
    std::vector<vct3> mMarkerPositions;

    double mReadTimeout;
    osaStopwatch mResponseTimer;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNDISerial);

#endif  // _mtsNDISerial_h
