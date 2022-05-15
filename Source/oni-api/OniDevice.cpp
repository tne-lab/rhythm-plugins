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

#include "OniDevice.hpp"


OniDevice::OniDevice()
{
    
}

OniDevice::~OniDevice()
{
    
}

ErrorCode OniDevice::open()
{
    
    // create the ONI context
    oni_ctx context = oni_create_ctx("ft600");
    
    ctx = &context;

    return SUCCESS;
}


ErrorCode OniDevice::close()
{
    // destroy the ONI context
    oni_destroy_ctx(*ctx);
    
    return SUCCESS;
}


ErrorCode OniDevice::initialize()
{
    // initialize the ONI context
    oni_init_ctx(*ctx, 0);
}


ErrorCode OniDevice::startAcquisition()
{
    int reg = 2;
    
    // reset the clock and automatically start acquisition
    oni_set_opt(*ctx, ONI_OPT_RESETACQCOUNTER, &reg, sizeof(oni_size_t));
}


ErrorCode OniDevice::stopAcquisition()
{
    
}


ErrorCode OniDevice::setParameter(DeviceParameter parameter, double value, int index)
{
    
}


std::vector<float> OniDevice::getAvailableSampleRates()
{
    
}


ErrorCode OniDevice::detectHeadstages(std::vector<RhythmNode::Headstage>&)
{
    
}


ErrorCode OniDevice::fillDataBlock(DataBlock* dataBlock)
{
    // Read a frame
    oni_frame_t *frame = NULL;
    oni_read_frame(*ctx, &frame);

    // Perform desired operations with frame
    
    // Frame structure:
    //struct oni_frame {
    //    const uint64_t dev_idx;  // Device index that produced or accepts the frame
    //    const uint32_t data_sz;  // Size in bytes of data buffer
    //    const uint64_t time;     // Frame time (ACQCLKHZ)
    //    uint8_t *data;           // Raw data block
    //};

    // Dispose of frame
    oni_destroy_frame(frame);
}


ErrorCode OniDevice::setDigitalOut(int* states, int numLines)
{
    
}


ErrorCode OniDevice::measureImpedances(Impedances&)
{
    
}
