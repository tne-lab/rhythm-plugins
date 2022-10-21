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

#include "RhythmDevice.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <chrono>
#include <cmath>


RhythmDevice::RhythmDevice(ControllerType type_, AmplifierSampleRate sampleRate_) :
    AbstractRHXController(type_, sampleRate_),
    chipRegisters(30000.0f)
{

    INIT_STEP = type_ == ControllerOEOpalKellyUSB3 ? 256 : 60;

    std::cout << "RhythmDevice constructor" << std::endl;
    std::cout << "Max num streams: " << maxNumDataStreams() << std::endl;
}

RhythmDevice::~RhythmDevice()
{

}

int RhythmDevice::open(const std::string& boardSerialNumber, const char* libraryFilePath)
{
    
    evalBoard = std::make_unique<Rhd2000EvalBoard>();

    int return_code = evalBoard->open(libraryFilePath);

    return return_code;
}


bool RhythmDevice::uploadFPGABitfile(const std::string& filename)
{

    return evalBoard->uploadFpgaBitfile(filename);
  
}
 
void RhythmDevice::resetBoard()
{
    evalBoard->resetBoard();
}


void RhythmDevice::run()
{
    evalBoard->run();
}

void RhythmDevice::updateRegisters()
{
    // Set up an RHD2000 register object using this sample rate to
    // optimize MUX-related register settings.
    chipRegisters.defineSampleRate(SampleRate30000Hz);

    int commandSequenceLength;
    std::vector<unsigned int> commandList;

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig(commandList, true);
    // Upload version with ADC calibration to AuxCmd3 RAM Bank 0.
    evalBoard->uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd3, 0,
        commandSequenceLength - 1);

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig(commandList, false);
    // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
    evalBoard->uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd3, 1);
    evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd3, 0,
        commandSequenceLength - 1);
}

bool RhythmDevice::isRunning()
{
    return evalBoard->isRunning();
}

void RhythmDevice::flush()
{
    evalBoard->flush();
}


void RhythmDevice::resetFpga()
{
    evalBoard->resetFpga();
}


bool RhythmDevice::readDataBlock(RHXDataBlock* dataBlock)
{

    evalBoard->readDataBlock(dataBlock);

    return true;
}


bool RhythmDevice::readDataBlocks(int numBlocks, std::deque<RHXDataBlock*>& dataQueue)
{

    return evalBoard->readDataBlocks(numBlocks, dataQueue);
}


long RhythmDevice::readDataBlocksRaw(int numBlocks, uint8_t* buffer)
{

    return evalBoard->readDataBlocksRaw(numBlocks, buffer);
}

bool RhythmDevice::readRawDataBlock(unsigned char** bufferPtr, int nSamples)
{
    return evalBoard->readRawDataBlock(bufferPtr, nSamples);
}

void RhythmDevice::setContinuousRunMode(bool continuousMode)
{

    evalBoard->setContinuousRunMode(continuousMode);
}

void RhythmDevice::setMaxTimeStep(unsigned int maxTimeStep)
{
    evalBoard->setMaxTimeStep(maxTimeStep);
}

void RhythmDevice::setCableDelay(BoardPort port, int delay)
{
    int bitShift = 0;

    if ((delay < 0) || (delay > 15)) {
        std::cerr << "Warning in RhythmDevice::setCableDelay: delay out of range: " << delay << '\n';
        if (delay < 0) delay = 0;
        else if (delay > 15) delay = 15;
    }

    evalBoard->setCableDelay(Rhd2000EvalBoard::BoardPort(port), delay);

}


void RhythmDevice::setDspSettle(bool enabled)
{
    evalBoard->setDspSettle(enabled);
   
}


void RhythmDevice::setTtlOut(const int* ttlOutArray)
{
 
    evalBoard->setTtlOut(ttlOutArray);
    
}


void RhythmDevice::setDacManual(int value)
{

    evalBoard->setDacManual(value);
}


void RhythmDevice::enableLeds(bool ledsOn)
{

    evalBoard->enableBoardLeds(ledsOn);
}

// Set output BNC clock divide factor (Open Ephys boards only)
void RhythmDevice::setClockDivider(int divide_factor)
{

    evalBoard->setClockDivider(divide_factor);

}


void RhythmDevice::setDacGain(int gain)
{

    evalBoard->setDacGain(gain);
}

// 
void RhythmDevice::setAudioNoiseSuppress(int noiseSuppress)
{

    evalBoard->setAudioNoiseSuppress(noiseSuppress);

}


void RhythmDevice::setExternalFastSettleChannel(int channel)
{

    evalBoard->setExternalFastSettleChannel(channel);
}


void RhythmDevice::setExternalDigOutChannel(BoardPort port, int channel)
{

    evalBoard->setExternalDigOutChannel(Rhd2000EvalBoard::BoardPort(port), channel);
}


void RhythmDevice::setDacHighpassFilter(double cutoff)
{

    evalBoard->setDacHighpassFilter(cutoff);
}


void RhythmDevice::setDacThreshold(int dacChannel, int threshold, bool trigPolarity)
{
    
    evalBoard->setDacThreshold(dacChannel, threshold, trigPolarity);

}


void RhythmDevice::setTtlMode(int mode)
{

    evalBoard->setTtlMode(mode);

}
 
void RhythmDevice::setDacRerefSource(int stream, int channel)
{

    std::cerr << "RhythmDevice::setDacRerefSource: not implemented." << std::endl;

}

bool RhythmDevice::setSampleRate(AmplifierSampleRate newSampleRate)
{

    evalBoard->setSampleRate(Rhd2000EvalBoard::SampleRate30000Hz);

    return true;
}


void RhythmDevice::enableDataStream(int stream, bool enabled)
{
    if (stream < 0 || stream > (maxNumDataStreams() - 1)) {
        std::cerr << "Error in RhythmDevice::enableDataStream: stream out of range.\n";
        return;
    }


    if (enabled) {
        if (dataStreamEnabled[stream] == 0) {
            evalBoard->enableDataStream(stream, enabled);
            dataStreamEnabled[stream] = 1;
            numDataStreams++;
        }
    }
    else {
        if (dataStreamEnabled[stream] == 1) {
            evalBoard->enableDataStream(stream, enabled);
            dataStreamEnabled[stream] = 0;
            numDataStreams--;
        }
    }
   
}


void RhythmDevice::enableDac(int dacChannel, bool enabled)
{

    evalBoard->enableDac(dacChannel, enabled);

}


void RhythmDevice::enableExternalFastSettle(bool enable)
{
    evalBoard->enableExternalFastSettle(enable);
}


void RhythmDevice::enableExternalDigOut(BoardPort port, bool enable)
{

    evalBoard->enableExternalDigOut(Rhd2000EvalBoard::BoardPort(port), enable);
   
}


void RhythmDevice::enableDacHighpassFilter(bool enable)
{

    evalBoard->enableDacHighpassFilter(enable);

}

void RhythmDevice::enableDacReref(bool enabled)
{

    std::cerr << "RhythmDevice::enableDacReref: not implemented." << std::endl;
}


void RhythmDevice::selectDacDataStream(int dacChannel, int stream)
{

    evalBoard->selectDacDataStream(dacChannel, stream);

}


void RhythmDevice::selectDacDataChannel(int dacChannel, int dataChannel)
{

    evalBoard->selectDacDataChannel(dacChannel, dataChannel);

}


void RhythmDevice::selectAuxCommandLength(AuxCmdSlot auxCommandSlot, int loopIndex, int endIndex)
{

    evalBoard->selectAuxCommandLength(Rhd2000EvalBoard::AuxCmdSlot(auxCommandSlot), loopIndex, endIndex);


}


void RhythmDevice::selectAuxCommandBank(BoardPort port, AuxCmdSlot auxCommandSlot, int bank)
{

    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::BoardPort(port), Rhd2000EvalBoard::AuxCmdSlot(auxCommandSlot), bank);

}

int RhythmDevice::getBoardMode()
{
    // not used?
    return 0; 
}


int RhythmDevice::getNumSPIPorts(bool& expanderBoardDetected)
{
    return 4;
}

 
void RhythmDevice::clearTtlOut()
{

    evalBoard->clearTtlOut();

}


void RhythmDevice::uploadCommandList(const std::vector<unsigned int>& commandList, AuxCmdSlot auxCommandSlot, int bank)
{

    evalBoard->uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmdSlot(auxCommandSlot), bank);
  
}

// Scan all SPI ports to find all connected RHD/RHS amplifier chips.  Read the chip ID from on-chip ROM
// register to determine the number of amplifier channels on each port.  This process is repeated at all
// possible MISO delays in the FPGA to determine the optimum MISO delay for each port to compensate for
// variable cable length.
//
// This function returns three vectors of length maxNumDataStreams(): chipType (the type of chip connected
// to each data stream); portIndex (the SPI port number [A=1, B=2,...] associated with each data stream);
// and commandStream (the stream index for sending commands to the FPGA for a particular read stream index).
// This function also returns a vector of length maxNumSPIPorts(): numChannelsOnPort (the total number of
// amplifier channels on each port).
//
// This function normally returns 1, but returns a negative number if a ControllerRecordUSB2 devices is used
// and its 256-channel capacity (limited by USB2 bus speed) is exceeded.  A value of -1 is returned, or a value
// of -2 if RHD2216 devices are present so that the user can be reminded that RHD2216 devices consume 32 channels
// of USB bus bandwidth.
int RhythmDevice::findConnectedChips(std::vector<ChipType>& chipType, std::vector<int>& portIndex, std::vector<int>& commandStream,
    std::vector<int>& numChannelsOnPort)
{
    int returnValue = 1;    // return 1 == everything okay
    int maxNumStreams = maxNumDataStreams();
    int maxSPIPorts = maxNumSPIPorts();
    int maxMISOLines = 2 * maxSPIPorts;

    chipType.resize(maxNumStreams);
    fill(chipType.begin(), chipType.end(), NoChip);
    std::vector<ChipType> chipTypeOld(maxNumStreams, NoChip);

    portIndex.resize(maxNumStreams);
    fill(portIndex.begin(), portIndex.end(), -1);
    std::vector<int> portIndexOld(maxMISOLines, -1);

    portIndexOld[0] = 0; portIndexOld[1] = 0;
    portIndexOld[2] = 1; portIndexOld[3] = 1;
    portIndexOld[4] = 2; portIndexOld[5] = 2;
    portIndexOld[6] = 3; portIndexOld[7] = 3;

    commandStream.resize(maxNumStreams);
    fill(commandStream.begin(), commandStream.end(), -1);

    numChannelsOnPort.resize(maxSPIPorts);
    fill(numChannelsOnPort.begin(), numChannelsOnPort.end(), 0);

    BoardDataSource initStreamPorts[8] = { PortA1, PortA2, PortB1, PortB2, PortC1, PortC2, PortD1, PortD2 };

    BoardDataSource initStreamDdrPorts[8] = { PortA1Ddr, PortA2Ddr, PortB1Ddr, PortB2Ddr,
                                              PortC1Ddr, PortC2Ddr, PortD1Ddr, PortD2Ddr };

    for (int i = 0; i < maxMISOLines; i++)
        evalBoard->setDataSource(i, Rhd2000EvalBoard::BoardDataSource(initStreamPorts[i]));

    for (int i = 0; i < maxMISOLines; i++)
        evalBoard->enableDataStream(i, true);

    // Run the SPI interface for multiple command sequences (i.e., NRepeats data blocks).
    const int NRepeats = 1;
    RHXDataBlock dataBlock(type, 8);
    setMaxTimeStep(NRepeats * dataBlock.samplesPerDataBlock());
    setContinuousRunMode(false);

    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortA,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortB,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortC,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortD,
        Rhd2000EvalBoard::AuxCmd3, 0);

    std::vector<std::vector<int> > goodDelays;
    goodDelays.resize(maxMISOLines);
    for (int i = 0; i < maxMISOLines; ++i) {
        goodDelays[i].resize(16);
        for (int j = 0; j < 16; ++j) {
            goodDelays[i][j] = 0;
        }
    }

    // Run SPI command sequence at all 16 possible FPGA MISO delay settings
    // to find optimum delay for each SPI interface cable.
    for (int delay = 0; delay < 16; ++delay) 
    {
        setCableDelay(PortA, delay);
        setCableDelay(PortB, delay);
        setCableDelay(PortC, delay);
        setCableDelay(PortD, delay);
        
        run();

        // Wait for the run to complete.
        while (isRunning()) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        for (int i = 0; i < NRepeats; ++i) {
            // Read one data block from the USB interface.
            readDataBlock(&dataBlock);

            // Read the Intan chip ID number from each RHD or RHS chip found.
            // Record delay settings that yield good communication with the chip.
            int register59Value;
            for (int stream = 0; stream < maxNumDataStreams(); stream++) {
                int id = dataBlock.getChipID(stream, Rhd2000EvalBoard::AuxCmd3, register59Value);
                if (id == (int)RHD2132Chip || id == (int)RHD2216Chip || id == (int)RHS2116Chip ||
                    (id == (int)RHD2164Chip && register59Value == Register59MISOA)) {
                    goodDelays[stream][delay] = goodDelays[stream][delay] + 1;
                    chipTypeOld[stream] = (ChipType)id;
                }
            }
        }
    }

    // Set cable delay settings that yield good communication with each chip.
    std::vector<int> optimumDelay(maxMISOLines, 0);
    for (int stream = 0; stream < maxMISOLines; ++stream) {
        int bestCount = -1;
        for (int delay = 0; delay < 16; ++delay) {
            if (goodDelays[stream][delay] > bestCount) {
                bestCount = goodDelays[stream][delay];
            }
        }
        int numBest = 0;
        for (int delay = 0; delay < 16; ++delay) {
            if (goodDelays[stream][delay] == bestCount) {
                ++numBest;
            }
        }
        int bestDelay = -1;
        if (numBest == 2 && (chipTypeOld[stream] == RHD2164Chip)) {
            for (int delay = 15; delay >= 0; --delay) {  // DDR SPI from RHD2164 chip seems to work best with longer of two valid delays.
                if (goodDelays[stream][delay] == bestCount) {
                    bestDelay = delay;
                    break;
                }
            }
        }
        else {
            for (int delay = 0; delay < 16; ++delay) {
                if (goodDelays[stream][delay] == bestCount) {
                    bestDelay = delay;
                    break;
                }
            }
            if (numBest > 2) {  // If 3 or more valid delays, don't use the longest or shortest.
                for (int delay = bestDelay + 1; delay < 16; ++delay) {
                    if (goodDelays[stream][delay] == bestCount) {
                        bestDelay = delay;
                        break;
                    }
                }
            }
        }

        optimumDelay[stream] = bestDelay;
    }

    setCableDelay(PortA, max(optimumDelay[0], optimumDelay[1]));
    setCableDelay(PortB, max(optimumDelay[2], optimumDelay[3]));
    setCableDelay(PortC, max(optimumDelay[4], optimumDelay[5]));
    setCableDelay(PortD, max(optimumDelay[6], optimumDelay[7]));

    if (type == ControllerRecordUSB3) {
        setCableDelay(PortE, max(optimumDelay[8], optimumDelay[9]));
        setCableDelay(PortF, max(optimumDelay[10], optimumDelay[11]));
        setCableDelay(PortG, max(optimumDelay[12], optimumDelay[13]));
        setCableDelay(PortH, max(optimumDelay[14], optimumDelay[15]));
    }

    // Now that we know which chips are plugged into each SPI port, add up the total number of
    // amplifier channels on each port and calcualate the number of data streams necessary to convey
    // this data over the USB interface.
    int numStreamsRequired = 0;
    bool rhd2216ChipPresent = false;
    for (int stream = 0; stream < maxMISOLines; stream++) {
        if (chipTypeOld[stream] == RHD2216Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 16;
            rhd2216ChipPresent = true;
        }
        if (chipTypeOld[stream] == RHD2132Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 32;
        }
        if (chipTypeOld[stream] == RHD2164Chip) {
            numStreamsRequired += 2;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 64;
        }
        if (chipTypeOld[stream] == RHS2116Chip) {
            numStreamsRequired++;
            if (numStreamsRequired <= maxNumStreams) numChannelsOnPort[portIndexOld[stream]] += 16;
        }
    }

    // Return a warning if 256-channel capacity of ControllerRecordUSB2 is exceeded.
    if ((type == ControllerRecordUSB2 || type == ControllerOEOpalKellyUSB2) && numStreamsRequired > 8) {
        returnValue = (rhd2216ChipPresent ? -2 : -1);   // -1 == ControllerRecordUSB2 has >256 channels plugged in.
                                                        // -2 == ...and at least one of the chips is an RHD2216.
    }

    // Reconfigure USB data streams in consecutive order to accommodate all connected chips.

    std::cout << "Reconfigure USB streams" << std::endl;

    for (int i = 0; i < maxMISOLines; i++)
        evalBoard->enableDataStream(i, false);

    if (type == ControllerOEOpalKellyUSB2) {
        int stream = 0;
        for (int oldStream = 0; oldStream < maxMISOLines; ++oldStream) {
            if ((chipTypeOld[oldStream] == RHD2216Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2216Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2132Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2132Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2164Chip) && (stream < maxNumStreams - 1)) {
                chipType[stream] = RHD2164Chip;
                chipType[stream + 1] = RHD2164MISOBChip;
                portIndex[stream] = portIndexOld[oldStream];
                portIndex[stream + 1] = portIndexOld[oldStream];
                enableDataStream(stream, true);
                enableDataStream(stream + 1, true);
                setDataSource(stream, initStreamPorts[oldStream]);
                setDataSource(stream + 1, initStreamDdrPorts[oldStream]);
                stream += 2;
            }
        }
    }
    else if (type == ControllerOEOpalKellyUSB3) {
        int stream = 0;
        for (int oldStream = 0; oldStream < maxMISOLines; ++oldStream) {
            if ((chipTypeOld[oldStream] == RHD2216Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2216Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, false);
                commandStream[stream] = 2 * oldStream;
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2132Chip) && (stream < maxNumStreams)) {
                chipType[stream] = RHD2132Chip;
                portIndex[stream] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, false);
                commandStream[stream] = 2 * oldStream;
                stream++;
            }
            else if ((chipTypeOld[oldStream] == RHD2164Chip) && (stream < maxNumStreams - 1)) {
                chipType[stream] = RHD2164Chip;
                chipType[stream + 1] = RHD2164MISOBChip;
                portIndex[stream] = portIndexOld[oldStream];
                portIndex[stream + 1] = portIndexOld[oldStream];
                enableDataStream(2 * oldStream, true);
                enableDataStream(2 * oldStream + 1, true);
                commandStream[stream] = 2 * oldStream;
                commandStream[stream + 1] = 2 * oldStream + 1;
                stream += 2;
            }
        }
    }

    std::cout << "Done reconfiguring USB streams" << std::endl;

    return returnValue;
}


unsigned int RhythmDevice::numWordsInFifo()
{

  
    return evalBoard->numWordsInFifo();
}


bool RhythmDevice::isDcmProgDone() const
{

    return evalBoard->isDcmProgDone();
}


bool RhythmDevice::isDataClockLocked() const
{

    return evalBoard->isDataClockLocked();
}


void RhythmDevice::forceAllDataStreamsOff()
{

    evalBoard->forceAllDataStreamsOff();
}

