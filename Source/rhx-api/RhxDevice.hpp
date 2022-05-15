/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2022 Open Ephys

    ------------------------------------------------------------------

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef RhxDevice_hpp
#define RhxDevice_hpp

#include <stdio.h>

#include "Abstract/abstractrhxcontroller.h"
#include "Hardware/rhxglobals.h"

#include "../AbstractDevice.hpp"

/**

    Class for communicating with Intan RHD/RHS devices
 
 */
class RhxDevice : public AbstractDevice
{
public:
    
    /** Constructor (0 = USB2, 1 = USB3, 2 = Stim/Record) */
    RhxDevice(int deviceType);
    
    /** Destructor */
    ~RhxDevice();
    
    /** Opens device, returns error code */
    ErrorCode open();
    
    /** Closes device, returns error code */
    ErrorCode close();
    
    /** Initializes device, returns error code*/
    ErrorCode initialize();
    
    /** Starts streaming data*/
    ErrorCode startAcquisition();
    
    /** Stops streaming data  */
    ErrorCode stopAcquisition();
    
    /** Sets a parameter */
    ErrorCode setParameter(DeviceParameter parameter, double value, int index = -1);
    
    /** Gets available sample rates */
    std::vector<float> getAvailableSampleRates();
    
    /** Updates the headstage array with connected headstage info */
    ErrorCode detectHeadstages(std::vector<RhythmNode::Headstage>&);
    
    /** Reads in a block of data*/
    ErrorCode fillDataBlock(DataBlock*);
    
    /** Sets state of digital output lines */
    ErrorCode setDigitalOut(int* states, int numLines = 8);
    
    /** Conducts impedance measurement (called from a separate thread)*/
    ErrorCode measureImpedances(Impedances&);
    
private:
    
    AbstractRHXController controller;
   
};

#endif /* RhxDevice_hpp */
