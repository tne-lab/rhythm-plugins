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

#ifndef RhythmDevice_hpp
#define RhythmDevice_hpp

#include <stdio.h>

#include "../AbstractDevice.hpp"

#include "rhd2000evalboard.h"
#include "rhd2000registers.h"
#include "rhd2000datablock.h"
#include "okFrontPanelDLL.h"

#define CHIP_ID_RHD2132  1
#define CHIP_ID_RHD2216  2
#define CHIP_ID_RHD2164  4
#define CHIP_ID_RHD2164_B  1000
#define REGISTER_59_MISO_A  53
#define REGISTER_59_MISO_B  58
#define RHD2132_16CH_OFFSET 8

class RhythmDevice : public AbstractDevice
{
    
public:
    /** Constructor */
    RhythmDevice();
    
    /** Destructor */
    ~RhythmDevice();
    
    /** Opens device, returns true if successful */
    ErrorCode open();
    
    /** Closes device, returns true if successful */
    ErrorCode close();
    
    /** Initializes device, returns true if successful */
    ErrorCode initialize();
    
    /** Starts streaming data*/
    ErrorCode startAcquisition();
    
    /** Stops streaming data  */
    ErrorCode stopAcquisition();
    
    /** Sets a parameter */
    ErrorCode setParameter(DeviceParameter parameter, double value);
    
    /** Returns an array of available headstages */
    ErrorCode detectHeadstages(std::vector<RhythmNode::Headstage>&);
    
    /** Gets available sample rates */
    std::vector<float> getAvailableSampleRates();
    
    /** Reads in a block of data*/
    ErrorCode fillDataBlock(DataBlock*);
    
private:
    
    /** Open the connection to the acquisition board*/
    ErrorCode openBoard();

    /** Upload the bitfile*/
    ErrorCode uploadBitfile();
    
    /** Sets the sample rate */
    void setSampleRate(int sampleRateIndex);
    
    /** Sets the cable length for a particular headstage*/
    void setCableLength(int hsNum, float length);
    
    /** Updates register values */
    void updateRegisters();
    
    /** Returns the device ID for an Intan chip*/
    int getDeviceId(Rhd2000DataBlock* dataBlock, int stream, int& register59Value);

    Rhd2000EvalBoard evalBoard;
    std::unique_ptr<Rhd2000DataBlock> rhd2000DataBlock;
    Rhd2000Registers chipRegisters;
    
    /** Data buffers*/
    float thisSample[MAX_NUM_CHANNELS];

    float auxBuffer[MAX_NUM_CHANNELS]; // aux inputs are only sampled every 4th sample, so use this to buffer the
                                       // samples so they can be handles just like the regular neural channels later

    float auxSamples[MAX_NUM_DATA_STREAMS][3];

    unsigned int blockSize;

    
    bool deviceFound = false;
    
};

#endif /* RhythmDevice_hpp */
