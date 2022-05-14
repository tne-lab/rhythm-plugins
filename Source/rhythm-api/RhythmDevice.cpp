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

#include "RhythmDevice.hpp"

#if defined(_WIN32)
#define okLIB_NAME "okFrontPanel.dll"
#define okLIB_EXTENSION "*.dll"
#elif defined(__APPLE__)
#define okLIB_NAME "libokFrontPanel.dylib"
#define okLIB_EXTENSION "*.dylib"
#elif defined(__linux__)
#define okLIB_NAME "./libokFrontPanel.so"
#define okLIB_EXTENSION "*.so"
#endif

#define INIT_STEP ( evalBoard.isUSB3() ? 256 : 60)


RhythmDevice::RhythmDevice()
 : chipRegisters(30000.0f)
{
    
}

RhythmDevice::~RhythmDevice()
{

}

ErrorCode RhythmDevice::open()
{
    
    errorCode = openBoard();
    
    if (errorCode != SUCCESS)
        return errorCode;
    
    errorCode = uploadBitfile();
    
    deviceFound = true;
    
    return errorCode;
    
}
    
ErrorCode RhythmDevice::openBoard()
{

#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation(File::currentApplicationFile);
    const String executableDirectory = appBundle.getChildFile("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation(File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    String libraryFilePath = executableDirectory;
    libraryFilePath += File::getSeparatorString();
    libraryFilePath += okLIB_NAME;

    int return_code = evalBoard.open(libraryFilePath.getCharPointer());

    if (return_code == -1) // dynamic library not found
    {
        return OPAL_KELLY_LIBRARY_NOT_FOUND;
    }
    else if (return_code == -2)   // board could not be opened
    {
        return NO_DEVICE_FOUND;
    }
    
    if (evalBoard.isUSB3())
        LOGD("USB3 board mode enabled");
    
    settings.samplesPerDataBlock = Rhd2000DataBlock::getSamplesPerDataBlock(evalBoard.isUSB3());
    
    int ledArray[8] = { 1, 0, 0, 0, 0, 0, 0, 0 };
    evalBoard.setLedDisplay(ledArray);
    
    rhd2000DataBlock = std::make_unique<Rhd2000DataBlock>(1, evalBoard.isUSB3());
    
}
    
ErrorCode RhythmDevice::uploadBitfile()
{
    String bitfilename;

#if defined(__APPLE__)
    File appBundle = File::getSpecialLocation(File::currentApplicationFile);
    const String executableDirectory = appBundle.getChildFile("Contents/Resources").getFullPathName();
#else
    File executable = File::getSpecialLocation(File::currentExecutableFile);
    const String executableDirectory = executable.getParentDirectory().getFullPathName();
#endif

    bitfilename = executableDirectory;
    bitfilename += File::getSeparatorString();
    bitfilename += "shared";
    bitfilename += File::getSeparatorString();

    bitfilename += evalBoard.isUSB3() ? "rhd2000_usb3.bit" : "rhd2000.bit";

    int returnCode = evalBoard.uploadFpgaBitfile(bitfilename.toStdString());
    
    if (returnCode != 1)
    {
        return BITFILE_NOT_FOUND;
    }
    
    return SUCCESS;
    
}

ErrorCode RhythmDevice::initialize()
{
    // Initialize the board
    LOGD("Initializing RHD2000 board.");
    evalBoard.initialize();
    // This applies the following settings:
    //  - sample rate to 30 kHz
    //  - aux command banks to zero
    //  - aux command lengths to zero
    //  - continuous run mode to 'true'
    //  - maxTimeStep to 2^32 - 1
    //  - all cable lengths to 3 feet
    //  - dspSettle to 'false'
    //  - data source mapping as 0->PortA1, 1->PortB1, 2->PortC1, 3->PortD1, etc.
    //  - enables all data streams
    //  - clears the ttlOut
    //  - disables all DACs and sets gain to 0

    setSampleRate(Rhd2000EvalBoard::SampleRate30000Hz);

    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortA, settings.cableLength.portA);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortB, settings.cableLength.portB);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortC, settings.cableLength.portC);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortD, settings.cableLength.portD);

    // Select RAM Bank 0 for AuxCmd3 initially, so the ADC is calibrated.
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3, 0);

    // Since our longest command sequence is 60 commands, run the SPI interface for
    // 60 samples (64 for usb3 power-of two needs)
    evalBoard.setMaxTimeStep(INIT_STEP);
    evalBoard.setContinuousRunMode(false);

    // Start SPI interface
    evalBoard.run();

    // Wait for the 60-sample run to complete
    while (evalBoard.isRunning())
    {
        ;
    }

    // Read the resulting single data block from the USB interface. We don't
    // need to do anything with this, since it was only used for ADC calibration
    ScopedPointer<Rhd2000DataBlock> dataBlock = new Rhd2000DataBlock(evalBoard.getNumEnabledDataStreams(), evalBoard.isUSB3());

    evalBoard.readDataBlock(dataBlock, INIT_STEP);
    // Now that ADC calibration has been performed, we switch to the command sequence
    // that does not execute ADC calibration.
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3,
        settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3,
        settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3,
        settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3,
        settings.fastSettleEnabled ? 2 : 1);

    return SUCCESS;

}

ErrorCode RhythmDevice::close()
{

    int ledArray[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    evalBoard.setLedDisplay(ledArray);
    
    if (deviceFound)
        evalBoard.resetFpga();
}


ErrorCode RhythmDevice::detectHeadstages(std::vector<RhythmNode::Headstage>& headstages)
{
    // scanPorts() function
}

int RhythmDevice::getDeviceId(Rhd2000DataBlock* dataBlock, int stream, int& register59Value)
{
    bool intanChipPresent;

    // First, check ROM registers 32-36 to verify that they hold 'INTAN', and
    // the initial chip name ROM registers 24-26 that hold 'RHD'.
    // This is just used to verify that we are getting good data over the SPI
    // communication channel.
    intanChipPresent = ((char) dataBlock->auxiliaryData[stream][2][32] == 'I' &&
                        (char) dataBlock->auxiliaryData[stream][2][33] == 'N' &&
                        (char) dataBlock->auxiliaryData[stream][2][34] == 'T' &&
                        (char) dataBlock->auxiliaryData[stream][2][35] == 'A' &&
                        (char) dataBlock->auxiliaryData[stream][2][36] == 'N' &&
                        (char) dataBlock->auxiliaryData[stream][2][24] == 'R' &&
                        (char) dataBlock->auxiliaryData[stream][2][25] == 'H' &&
                        (char) dataBlock->auxiliaryData[stream][2][26] == 'D');

    // If the SPI communication is bad, return -1.  Otherwise, return the Intan
    // chip ID number stored in ROM regstier 63.
    if (!intanChipPresent)
    {
        register59Value = -1;
        return -1;
    }
    else
    {
        register59Value = dataBlock->auxiliaryData[stream][2][23]; // Register 59
        return dataBlock->auxiliaryData[stream][2][19]; // chip ID (Register 63)
    }
}

std::vector<float> RhythmDevice::getAvailableSampleRates()
{
    std::vector<float> sampleRates;
    
    sampleRates.push_back(1000.0f);
    sampleRates.push_back(1250.0f);
    sampleRates.push_back(1500.0f);
    sampleRates.push_back(2000.0f);
    sampleRates.push_back(2500.0f);
    sampleRates.push_back(3000.0f);
    sampleRates.push_back(3333.0f);
    sampleRates.push_back(4000.0f);
    sampleRates.push_back(5000.0f);
    sampleRates.push_back(6250.0f);
    sampleRates.push_back(8000.0f);
    sampleRates.push_back(10000.0f);
    sampleRates.push_back(12500.0f);
    sampleRates.push_back(15000.0f);
    sampleRates.push_back(20000.0f);
    sampleRates.push_back(25000.0f);
    sampleRates.push_back(30000.0f);
    
    return sampleRates;
}

    
ErrorCode RhythmDevice::startAcquisition()
{
    if (!deviceFound)
        return FAILED_TO_START_ACQUISITION;
    
    rhd2000DataBlock.reset();
    
    rhd2000DataBlock = std::make_unique<Rhd2000DataBlock>(evalBoard.getNumEnabledDataStreams(), evalBoard.isUSB3());
    
    blockSize = rhd2000DataBlock->calculateDataBlockSizeInWords(evalBoard.getNumEnabledDataStreams(), evalBoard.isUSB3());

    int ledArray[8] = {1, 1, 0, 0, 0, 0, 0, 0};
    evalBoard.setLedDisplay(ledArray);

    evalBoard.flush();
    evalBoard.setContinuousRunMode(true);
    evalBoard.run();
    
    return SUCCESS;
}

ErrorCode RhythmDevice::stopAcquisition()
{
    
    LOGD( "Number of 16-bit words in FIFO: ", evalBoard.numWordsInFifo() );
    
    evalBoard.setContinuousRunMode(false);
    evalBoard.setMaxTimeStep(0);
    LOGD( "Flushing FIFO." );
    evalBoard.flush();
    
    int ledArray[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    evalBoard.setLedDisplay(ledArray);
    
    return SUCCESS;

}

ErrorCode RhythmDevice::fillDataBlock(DataBlock* dataBlock)
{

    unsigned char* bufferPtr;
    double ts;
    
    if (evalBoard.isUSB3() || evalBoard.numWordsInFifo() >= blockSize)
    {
        bool return_code;

        return_code = evalBoard.readRawDataBlock(&bufferPtr);

        int index = 0;
        int auxIndex, chanIndex;
        int numStreams = enabledStreams.size();
        int nSamps = settings.samplesPerDataBlock;

        for (int samp = 0; samp < nSamps; samp++)
        {
            int channel = -1;

            if (!Rhd2000DataBlock::checkUsbHeader(bufferPtr, index))
            {
                LOGE( "Error in Rhd2000EvalBoard::readDataBlock: Incorrect header." );
                break;
            }

            index += 8; // magic number header width (bytes)
            dataBlock->sampleNumbers[samp] = Rhd2000DataBlock::convertUsbTimeStamp(bufferPtr, index);
            
            index += 4; // timestamp width
            auxIndex = index; // aux chans start at this offset
            
            // skip aux channels for now
            index += 6 * numStreams; // width of the 3 aux chans

            for (int dataStream = 0; dataStream < numStreams; dataStream++)
            {
                
                int nChans = numChannelsPerDataStream[dataStream];
                chanIndex = index + 2*dataStream;
                
                if ((chipId[dataStream] == CHIP_ID_RHD2132) && (nChans == 16)) //RHD2132 16ch. headstage
                {
                    chanIndex += 2 * RHD2132_16CH_OFFSET*numStreams;
                }
                for (int chan = 0; chan < nChans; chan++)
                {
                    channel++;
                    thisSample[channel] = float(*(uint16*)(bufferPtr + chanIndex) - 32768)*0.195f;
                    chanIndex += 2*numStreams; // single chan width (2 bytes)
                }
            }
            
            index += 64 * numStreams; // neural data width
            auxIndex += 2 * numStreams; // skip AuxCmd1 slots (see updateRegisters())
            // copy the 3 aux channels
            
            if (settings.acquireAux)
            {
                for (int dataStream = 0; dataStream < numStreams; dataStream++)
                {
                    if (chipId[dataStream] != CHIP_ID_RHD2164_B)
                    {
                        int auxNum = (samp+3) % 4;
                        if (auxNum < 3)
                        {
                            auxSamples[dataStream][auxNum] = float(*(uint16*)(bufferPtr + auxIndex) - 32768)*0.0000374;
                        }
                        for (int chan = 0; chan < 3; chan++)
                        {
                            channel++;
                            if (auxNum == 3)
                            {
                                auxBuffer[channel] = auxSamples[dataStream][chan];
                            }
                            thisSample[channel] = auxBuffer[channel];
                        }
                    }
                    auxIndex += 2; // single chan width (2 bytes)
                }
            }
            
            index += 2 * numStreams; // skip over filler word at the end of each data stream
            // copy the 8 ADC channels
            if (settings.acquireAdc)
            {
                for (int adcChan = 0; adcChan < 8; ++adcChan)
                {

                    channel++;
                    // ADC waveform units = volts

                    if (boardType == ACQUISITION_BOARD)
                    {
                        thisSample[channel] = adcRangeSettings[adcChan] == 0 ?
                            0.00015258789 * float(*(uint16*)(bufferPtr + index)) - 5 - 0.4096 : // account for +/-5V input range and DC offset
                            0.00030517578 * float(*(uint16*)(bufferPtr + index)); // shouldn't this be half the value, not 2x?
                    }
                    else if (boardType == INTAN_RHD_USB) {
                        thisSample[channel] = 0.000050354 * float(dataBlock->boardAdcData[adcChan][samp]);
                    }
                    index += 2; // single chan width (2 bytes)
                }
            }
            else
            {
                index += 16; // skip ADC chans (8 * 2 bytes)
            }

            uint64 ttlEventWord = *(uint64*)(bufferPtr + index) & 65535;

            index += 4;
        }
    }
}


ErrorCode RhythmDevice::setParameter(DeviceParameter parameter, double value)
{
    if (parameter == SAMPLE_RATE)
    {
        setSampleRate(value);
    }
}

void RhythmDevice::setSampleRate(int sampleRateIndex)
{
    int numUsbBlocksToRead = 0; // placeholder - make this change the number of blocks that are read in DeviceThread::updateBuffer()

    Rhd2000EvalBoard::AmplifierSampleRate sampleRate; // just for local use

    switch (sampleRateIndex)
    {
        case 0:
            sampleRate = Rhd2000EvalBoard::SampleRate1000Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1000.0f;
            break;
        case 1:
            sampleRate = Rhd2000EvalBoard::SampleRate1250Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1250.0f;
            break;
        case 2:
            sampleRate = Rhd2000EvalBoard::SampleRate1500Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 1500.0f;
            break;
        case 3:
            sampleRate = Rhd2000EvalBoard::SampleRate2000Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 2000.0f;
            break;
        case 4:
            sampleRate = Rhd2000EvalBoard::SampleRate2500Hz;
            numUsbBlocksToRead = 1;
            settings.boardSampleRate = 2500.0f;
            break;
        case 5:
            sampleRate = Rhd2000EvalBoard::SampleRate3000Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 3000.0f;
            break;
        case 6:
            sampleRate = Rhd2000EvalBoard::SampleRate3333Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 3333.0f;
            break;
        case 7:
            sampleRate = Rhd2000EvalBoard::SampleRate4000Hz;
            numUsbBlocksToRead = 2;
            settings.boardSampleRate = 4000.0f;
            break;
        case 8:
            sampleRate = Rhd2000EvalBoard::SampleRate5000Hz;
            numUsbBlocksToRead = 3;
            settings.boardSampleRate = 5000.0f;
            break;
        case 9:
            sampleRate = Rhd2000EvalBoard::SampleRate6250Hz;
            numUsbBlocksToRead = 3;
            settings.boardSampleRate = 6250.0f;
            break;
        case 10:
            sampleRate = Rhd2000EvalBoard::SampleRate8000Hz;
            numUsbBlocksToRead = 4;
            settings.boardSampleRate = 8000.0f;
            break;
        case 11:
            sampleRate = Rhd2000EvalBoard::SampleRate10000Hz;
            numUsbBlocksToRead = 6;
            settings.boardSampleRate = 10000.0f;
            break;
        case 12:
            sampleRate = Rhd2000EvalBoard::SampleRate12500Hz;
            numUsbBlocksToRead = 7;
            settings.boardSampleRate = 12500.0f;
            break;
        case 13:
            sampleRate = Rhd2000EvalBoard::SampleRate15000Hz;
            numUsbBlocksToRead = 8;
            settings.boardSampleRate = 15000.0f;
            break;
        case 14:
            sampleRate = Rhd2000EvalBoard::SampleRate20000Hz;
            numUsbBlocksToRead = 12;
            settings.boardSampleRate = 20000.0f;
            break;
        case 15:
            sampleRate = Rhd2000EvalBoard::SampleRate25000Hz;
            numUsbBlocksToRead = 14;
            settings.boardSampleRate = 25000.0f;
            break;
        case 16:
            sampleRate = Rhd2000EvalBoard::SampleRate30000Hz;
            numUsbBlocksToRead = 16;
            settings.boardSampleRate = 30000.0f;
            break;
        default:
            sampleRate = Rhd2000EvalBoard::SampleRate10000Hz;
            numUsbBlocksToRead = 6;
            settings.boardSampleRate = 10000.0f;
    }


    // Select per-channel amplifier sampling rate.
    evalBoard.setSampleRate(sampleRate);

    LOGD( "Sample rate set to ", evalBoard.getSampleRate() );

    // Now that we have set our sampling rate, we can set the MISO sampling delay
    // which is dependent on the sample rate.
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortA, settings.cableLength.portA);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortB, settings.cableLength.portB);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortC, settings.cableLength.portC);
    evalBoard.setCableLengthMeters(Rhd2000EvalBoard::PortD, settings.cableLength.portD);

    updateRegisters();

    
}


void RhythmDevice::updateRegisters()
{
    if (!deviceFound) //Safety to avoid crashes loading a chain with Rythm node withouth a board
    {
        return;
    }
    // Set up an RHD2000 register object using this sample rate to
    // optimize MUX-related register settings.
    chipRegisters.defineSampleRate(settings.boardSampleRate);

    int commandSequenceLength;
    std::vector<int> commandList;

    // Create a command list for the AuxCmd1 slot.  This command sequence will continuously
    // update Register 3, which controls the auxiliary digital output pin on each RHD2000 chip.
    // In concert with the v1.4 Rhythm FPGA code, this permits real-time control of the digital
    // output pin on chips on each SPI port.
    chipRegisters.setDigOutLow();   // Take auxiliary output out of HiZ mode.
    commandSequenceLength = chipRegisters.createCommandListUpdateDigOut(commandList);
    evalBoard.uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard.selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd1, 0, commandSequenceLength - 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd1, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd1, 0);

    // Next, we'll create a command list for the AuxCmd2 slot.  This command sequence
    // will sample the temperature sensor and other auxiliary ADC inputs.
    commandSequenceLength = chipRegisters.createCommandListTempSensor(commandList);
    evalBoard.uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard.selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd2, 0, commandSequenceLength - 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd2, 0);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd2, 0);

    // Before generating register configuration command sequences, set amplifier
    // bandwidth paramters.
    settings.dsp.cutoffFreq = chipRegisters.setDspCutoffFreq(settings.dsp.cutoffFreq);
    settings.dsp.lowerBandwidth = chipRegisters.setLowerBandwidth(settings.dsp.lowerBandwidth);
    settings.dsp.upperBandwidth = chipRegisters.setUpperBandwidth(settings.dsp.upperBandwidth);
    chipRegisters.enableDsp(settings.dsp.enabled);

    // enable/disable aux inputs:
    chipRegisters.enableAux1(settings.acquireAux);
    chipRegisters.enableAux2(settings.acquireAux);
    chipRegisters.enableAux3(settings.acquireAux);

    chipRegisters.createCommandListRegisterConfig(commandList, true);
    // Upload version with ADC calibration to AuxCmd3 RAM Bank 0.
    evalBoard.uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard.selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd3, 0,
                                      commandSequenceLength - 1);

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig(commandList, false);
    // Upload version with no ADC calibration to AuxCmd3 RAM Bank 1.
    evalBoard.uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd3, 1);
    evalBoard.selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd3, 0,
                                      commandSequenceLength - 1);


    chipRegisters.setFastSettle(true);

    commandSequenceLength = chipRegisters.createCommandListRegisterConfig(commandList, false);
    // Upload version with fast settle enabled to AuxCmd3 RAM Bank 2.
    evalBoard.uploadCommandList(commandList, Rhd2000EvalBoard::AuxCmd3, 2);
    evalBoard.selectAuxCommandLength(Rhd2000EvalBoard::AuxCmd3, 0,
                                      commandSequenceLength - 1);

    chipRegisters.setFastSettle(false);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortA, Rhd2000EvalBoard::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortB, Rhd2000EvalBoard::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortC, Rhd2000EvalBoard::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);
    evalBoard.selectAuxCommandBank(Rhd2000EvalBoard::PortD, Rhd2000EvalBoard::AuxCmd3,
                                    settings.fastSettleEnabled ? 2 : 1);

}

void RhythmDevice::setCableLength(int hsNum, float length)
{
    // Set the MISO sampling delay, which is dependent on the sample rate.

    switch (hsNum)
    {
        case 0:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortA, length);
            break;
        case 1:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortB, length);
            break;
        case 2:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortC, length);
            break;
        case 3:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortD, length);
            break;
        case 4:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortE, length);
            break;
        case 5:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortF, length);
            break;
        case 6:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortG, length);
            break;
        case 7:
            evalBoard.setCableLengthFeet(Rhd2000EvalBoard::PortH, length);
            break;
        default:
            break;
    }

}
