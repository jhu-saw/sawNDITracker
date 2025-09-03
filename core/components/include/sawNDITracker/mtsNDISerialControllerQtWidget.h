/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsNDISerialControllerQtWidget_h
#define _mtsNDISerialControllerQtWidget_h

#include <cisstMultiTask/mtsComponent.h>

#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>
#include <QGridLayout>

// Always include last
#include <sawNDITracker/sawNDITrackerQtExport.h>

class CISST_EXPORT mtsNDISerialControllerQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsNDISerialControllerQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsNDISerialControllerQtWidget() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

 protected:
    virtual void closeEvent(QCloseEvent * event);

 private slots:
    void timerEvent(QTimerEvent * event);

 private:
    //! setup GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

 protected:
    std::string mSerialPort;

    struct {
        mtsFunctionRead  period_statistics;
        mtsFunctionWrite Connect;
        mtsFunctionVoid  Disconnect;
        mtsFunctionVoid  InitializeAll;
        mtsFunctionRead  Name;
        mtsFunctionWrite Track;
        mtsFunctionWrite TrackStrayMarkers;
        mtsFunctionWrite Beep;
    } Tracker;

 private:
    // Control
    QCheckBox * QCBConnect;
    QPushButton * QPBInitializeAll;
    QLabel * QLPortName;
    QCheckBox * QCBTrack;
    QCheckBox * QCBTrackStrayMarkers;
    QPushButton * QPBBeepButton;
    QSpinBox * QSBBeepCount;

    // Timing
    mtsIntervalStatistics IntervalStatistics;
    mtsIntervalStatisticsQtWidget * QMIntervalStatistics;

    // Messages
    mtsMessageQtWidget * QMMessage;

    void SetControlWidgetsEnabled(const bool enabled);

 private slots:
    void SlotConnect(bool);
    void SlotInitializeAll(void);
    void SlotTrack(bool);
    void SlotTrackStrayMarkers(bool);
    void SlotBeep(void);

    void SlotConnectedEvent(void);

 signals:
    void SignalConnectedEvent(void);

 private:
    void ConnectedEventHandler(const std::string & connected);
    void TrackingEventHandler(const bool & tracking);
    void TrackingStrayMarkersEventHandler(const bool & tracking);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsNDISerialControllerQtWidget);

#endif // _mtsNDISerialControllerQtWidget_h
