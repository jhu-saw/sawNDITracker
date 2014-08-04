import os
import platform
import sys
import time

from cisstCommonPython import *
from cisstVectorPython import *
from cisstOSAbstractionPython import *
from cisstMultiTaskPython import *
from cisstParameterTypesPython import *
from cisstStereoVisionPython import *

name = 'My Tracker'
period = 0.01
#load the NDI config file located in the same directory as this python script.
configuration = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'configNDITracker.xml')

manager = mtsManagerLocal.GetInstance()
manager.CreateAllAndWait(5.0)
manager.StartAllAndWait(5.0)

proxy = mtsComponentWithManagement('{}Proxy'.format(name))
manager.AddComponent(proxy)
proxy.CreateAndWait(5.0)
time.sleep(0.5)

services = proxy.GetManagerComponentServices()

#create an instance of the tracker
result = services.Load('sawNDITracker')
assert result, 'Failed to load {} using component services'.format('sawNDITracker')

args = mtsTaskPeriodicConstructorArg(name, period)
result = services.ComponentCreate('mtsNDISerial', args)
assert result, 'Failed to create {} of type {}'.format(name, 'mtsNDISerial')

component = manager.GetComponent(name)
component.Configure(configuration)

#create the main interface to the tracker
controller = proxy.AddInterfaceRequiredAndConnect((name, 'Controller'))

component.CreateAndWait(5.0)
component.StartAndWait(5.0)

#see if the device is actually connected
controller.Beep(1)
time.sleep(1.0)
controller.Beep(2)

#initialize controller
controller.PortHandlesInitialize()
controller.PortHandlesQuery()
controller.PortHandlesEnable()
time.sleep(0.5)

#create an interface for the tracked pointer body
trackedBodyName = 'Pointer'
trackedBody = proxy.AddInterfaceRequiredAndConnect((name, trackedBodyName))

#enable tracking
controller.ToggleTracking(True)
controller.Beep(2)

print 'Controller tracking status: ' + str(controller.IsTracking())

while True:
    pose = trackedBody.GetPositionCartesian()
    if pose.GetValid():  # if visible
        print pose.Position().Translation()
    else: 
        print trackedBodyName + 'is not visible'
