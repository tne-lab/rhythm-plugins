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

#include "RhxDevice.hpp"

RhxDevice::RhxDevice(int deviceType)
    : controller(ControllerType(deviceType), SampleRate30000Hz)
{
    
}

RhxDevice::~RhxDevice()
{
    
}

ErrorCode RhxDevice::open()
{
    
}


ErrorCode RhxDevice::close()
{
    
}


ErrorCode RhxDevice::initialize()
{
    
}


ErrorCode RhxDevice::startAcquisition()
{
    
}


ErrorCode RhxDevice::stopAcquisition()
{
    
}


ErrorCode RhxDevice::setParameter(DeviceParameter parameter, double value, int index)
{
    
}


std::vector<float> RhxDevice::getAvailableSampleRates()
{
    
}


ErrorCode RhxDevice::detectHeadstages(std::vector<RhythmNode::Headstage>&)
{
    
}


ErrorCode RhxDevice::fillDataBlock(DataBlock*)
{
    
}


ErrorCode RhxDevice::setDigitalOut(int* states, int numLines)
{
    
}


ErrorCode RhxDevice::measureImpedances(Impedances&)
{
    
}
