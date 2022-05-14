/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2016 Open Ephys

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

#ifdef _WIN32
#define NOMINMAX
#endif

#include "DeviceThread.h"
#include "DeviceEditor.h"

#include "ImpedanceMeter.h"
#include "Headstage.h"

#include "rhythm-api/RhythmDevice.hpp"

using namespace RhythmNode;

BoardType DeviceThread::boardType = ACQUISITION_BOARD; // initialize static member

DataThread* DeviceThread::createDataThread(SourceNode *sn)
{
    return new DeviceThread(sn, boardType);
}

DeviceThread::DeviceThread(SourceNode* sn, BoardType boardType_) : DataThread(sn),
    deviceFound(false),
    isTransmitting(false),
    channelNamingScheme(GLOBAL_INDEX),
    updateSettingsDuringAcquisition(false)
{

    boardType = boardType_;

    impedanceThread = new ImpedanceMeter(this);

    memset(auxBuffer, 0, sizeof(auxBuffer));
    memset(auxSamples, 0, sizeof(auxSamples));

    for (int i = 0; i < 8; i++)
        adcRangeSettings[i] = 0;

    int maxNumHeadstages = (boardType == RHD_RECORDING_CONTROLLER) ? 16 : 8;

    for (int i = 0; i < maxNumHeadstages; i++)
        headstages.add(new Headstage(static_cast<Rhd2000EvalBoard::BoardDataSource>(i)));

    if (boardType == ACQUISITION_BOARD)
        device = std::make_unique<RhythmDevice>();
    
    sourceBuffers.add(new DataBuffer(2, 10000)); // start with 2 channels and automatically resize

    // Open Opal Kelly XEM6010 board.
    // Returns 1 if successful, -1 if FrontPanel cannot be loaded, and -2 if XEM6010 can't be found.

    
    dacStream = new int[8];
    dacChannels = new int[8];
    dacThresholds = new float[8];
    dacChannelsToUpdate = new bool[8];
    
    // 1. attempt to open the device
    ErrorCode errorCode = device->open();
    
    if (errorCode == OPAL_KELLY_LIBRARY_NOT_FOUND)
    {
        AlertWindow::showMessageBox(AlertWindow::NoIcon,
                                                     "Opal Kelly library not found.",
                                                     "The Opal Kelly library file was not found in the directory of the executable. Please locate it and re-add the plugin to the signal chain."
                                                     "Ok.");
    } else if (errorCode == NO_DEVICE_FOUND)
    {
        AlertWindow::showMessageBox(AlertWindow::NoIcon,
                                                     "No device was found.",
                                                     "Please connect one and re-add the plugin to the signal chain."
                                                     "Ok.");
    } else if (errorCode == BITFILE_NOT_FOUND)
    {
        AlertWindow::showMessageBox(AlertWindow::NoIcon,
                                                     "No bitfile was found.",
                                                     "Please place one in the appropriate directory and re-add the plugin to the signal chain."
                                                     "Ok.");
    } else if (errorCode == SUCCESS)
    {
        errorCode = device->detectHeadstages(headstages);
        
        for (int k = 0; k < 8; k++)
        {
            dacChannelsToUpdate[k] = true;
            dacStream[k] = 0;
            setDACthreshold(k, 65534);
            dacChannels[k] = 0;
            dacThresholds[k] = 0;
        }
    }
    
}

DeviceThread::~DeviceThread()
{

    LOGD("Closing device.");
    
    device->close();

    delete[] dacStream;
    delete[] dacChannels;
    delete[] dacThresholds;
    delete[] dacChannelsToUpdate;
}

void DeviceThread::initialize(bool signalChainIsLoading)
{
    
    LOGD("Initializing device.");
    
    device->initialize();
    
    adcChannelNames.clear();
    ttlLineNames.clear();

    for (int i = 0; i < 8; i++)
    {
        adcChannelNames.add("ADC" + String(i + 1));
        ttlLineNames.add("TTL" + String(i + 1));
    }
}

std::unique_ptr<GenericEditor> DeviceThread::createEditor(SourceNode* sn)
{

    std::unique_ptr<DeviceEditor> editor = std::make_unique<DeviceEditor>(sn, this);

    return editor;
}

void DeviceThread::handleBroadcastMessage(String msg)
{
    StringArray parts = StringArray::fromTokens(msg, " ", "");

    //std::cout << "Received " << msg << std::endl;

    if (parts[0].equalsIgnoreCase("ACQBOARD"))
    {
        if (parts.size() > 1)
        {
            String command = parts[1];

            if (command.equalsIgnoreCase("TRIGGER"))
            {
                if (parts.size() == 4)
                {
                    int ttlLine = parts[2].getIntValue() - 1;

                    if (ttlLine < 0 || ttlLine > 7)
                        return;

                    int eventDurationMs = parts[3].getIntValue();

                    if (eventDurationMs < 10 || eventDurationMs > 5000)
                        return;

                    DigitalOutputCommand command;
                    command.ttlLine = ttlLine;
                    command.state = true;

                    digitalOutputCommands.push(command);

                    DigitalOutputTimer* timer = new DigitalOutputTimer(this, ttlLine, eventDurationMs);

                    digitalOutputTimers.add(timer);

                }
            }
        }
    }

}


void DeviceThread::addDigitalOutputCommand(DigitalOutputTimer* timerToDelete, int ttlLine, bool state)
{
    DigitalOutputCommand command;
    command.ttlLine = ttlLine;
    command.state = state;

    digitalOutputCommands.push(command);

    digitalOutputTimers.removeObject(timerToDelete);
}

DeviceThread::DigitalOutputTimer::DigitalOutputTimer(DeviceThread* board_, int ttlLine_, int eventDurationMs)
    : board(board_)
{

    tllOutputLine = ttlLine_;

    startTimer(eventDurationMs);
}

void DeviceThread::DigitalOutputTimer::timerCallback()
{
    stopTimer();

    board->addDigitalOutputCommand(this, tllOutputLine, false);
}

void DeviceThread::setDACthreshold(int dacOutput, float threshold)
{
    dacThresholds[dacOutput]= threshold;
    dacChannelsToUpdate[dacOutput] = true;
    updateSettingsDuringAcquisition = true;

    //evalBoard->setDacThresholdVoltage(dacOutput,threshold);
}

void DeviceThread::setDACchannel(int dacOutput, int channel)
{
    if (channel < getNumDataOutputs(ContinuousChannel::ELECTRODE))
    {
        int channelCount = 0;
        for (int i = 0; i < enabledStreams.size(); i++)
        {
            if (channel < channelCount + numChannelsPerDataStream[i])
            {
                dacChannels[dacOutput] = channel - channelCount;
                dacStream[dacOutput] = i;
                break;
            }
            else
            {
                channelCount += numChannelsPerDataStream[i];
            }
        }
        dacChannelsToUpdate[dacOutput] = true;
        updateSettingsDuringAcquisition = true;
    }
}

Array<int> DeviceThread::getDACchannels() const
{
    Array<int> dacChannelsArray;

    for (int k = 0; k < 8; ++k)
    {
        dacChannelsArray.add (dacChannels[k]);
    }

    return dacChannelsArray;
}


void DeviceThread::scanPorts()
{
    if (!deviceFound) //Safety to avoid crashes if board not present
    {
        return;
    }

    impedanceThread->stopThreadSafely();

    //Clear previous known streams
    enabledStreams.clear();

    // Scan SPI ports
    int delay, hs, id;
    int register59Value;

    Rhd2000EvalBoard::BoardDataSource initStreamPorts[8] =
    {
        Rhd2000EvalBoard::PortA1,
        Rhd2000EvalBoard::PortA2,
        Rhd2000EvalBoard::PortB1,
        Rhd2000EvalBoard::PortB2,
        Rhd2000EvalBoard::PortC1,
        Rhd2000EvalBoard::PortC2,
        Rhd2000EvalBoard::PortD1,
        Rhd2000EvalBoard::PortD2
        //Rhd2000EvalBoard::PortE1,
        //Rhd2000EvalBoard::PortE2,
        //Rhd2000EvalBoard::PortF1,
        //Rhd2000EvalBoard::PortF2,
        //Rhd2000EvalBoard::PortG1,
        //Rhd2000EvalBoard::PortG2,
        //Rhd2000EvalBoard::PortH1,
        //Rhd2000EvalBoard::PortH2
    };

    chipId.insertMultiple(0, -1, 8);
    Array<int> tmpChipId(chipId);

    setSampleRate(Rhd2000EvalBoard::SampleRate30000Hz, true); // set to 30 kHz temporarily

    // Enable all data streams, and set sources to cover one or two chips
    // on Ports A-D.

    // THIS IS DIFFERENT FOR RECORDING CONTROLLER:
    for (int i = 0; i < 8; i++)
        evalBoard->setDataSource(i, initStreamPorts[i]);

    for (int i = 0; i < 8; i++)
        evalBoard->enableDataStream(i, true);

    LOGD("Number of enabled data streams: ", evalBoard->getNumEnabledDataStreams());

    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortA,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortB,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortC,
        Rhd2000EvalBoard::AuxCmd3, 0);
    evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortD,
        Rhd2000EvalBoard::AuxCmd3, 0);

    if (boardType == RHD_RECORDING_CONTROLLER)
    {
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortE,
            Rhd2000EvalBoard::AuxCmd3, 0);
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortF,
            Rhd2000EvalBoard::AuxCmd3, 0);
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortG,
            Rhd2000EvalBoard::AuxCmd3, 0);
        evalBoard->selectAuxCommandBank(Rhd2000EvalBoard::PortH,
            Rhd2000EvalBoard::AuxCmd3, 0);
    }

    // Since our longest command sequence is 60 commands, we run the SPI
    // interface for 60 samples. (64 for usb3 power-of two needs)
    evalBoard->setMaxTimeStep(INIT_STEP);
    evalBoard->setContinuousRunMode(false);

    ScopedPointer<Rhd2000DataBlock> dataBlock =
        new Rhd2000DataBlock(evalBoard->getNumEnabledDataStreams(), evalBoard->isUSB3());

    Array<int> sumGoodDelays;
    sumGoodDelays.insertMultiple(0, 0, 8);

    Array<int> indexFirstGoodDelay;
    indexFirstGoodDelay.insertMultiple(0, -1, 8);

    Array<int> indexSecondGoodDelay;
    indexSecondGoodDelay.insertMultiple(0, -1, 8);

    // Run SPI command sequence at all 16 possible FPGA MISO delay settings
    // to find optimum delay for each SPI interface cable.

    LOGD( "Checking for connected amplifier chips..." );

    for (delay = 0; delay < 16; delay++)
    {
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortA, delay);
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortB, delay);
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortC, delay);
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortD, delay);

        if (boardType == RHD_RECORDING_CONTROLLER)
        {
            evalBoard->setCableDelay(Rhd2000EvalBoard::PortE, delay);
            evalBoard->setCableDelay(Rhd2000EvalBoard::PortF, delay);
            evalBoard->setCableDelay(Rhd2000EvalBoard::PortG, delay);
            evalBoard->setCableDelay(Rhd2000EvalBoard::PortH, delay);
        }

        // Start SPI interface.
        evalBoard->run();

        // Wait for the 60-sample run to complete.
        while (evalBoard->isRunning())
        {
            ;
        }
        // Read the resulting single data block from the USB interface.
        evalBoard->readDataBlock(dataBlock, INIT_STEP);

        // Read the Intan chip ID number from each RHD2000 chip found.
        // Record delay settings that yield good communication with the chip.
        for (hs = 0; hs < headstages.size(); ++hs)
        {

            id = getDeviceId(dataBlock, hs, register59Value);

            if (id == CHIP_ID_RHD2132 || id == CHIP_ID_RHD2216 ||
                (id == CHIP_ID_RHD2164 && register59Value == REGISTER_59_MISO_A))
            {
                LOGD( "Device ID found: ", id );

                sumGoodDelays.set(hs, sumGoodDelays[hs] + 1);

                if (indexFirstGoodDelay[hs] == -1)
                {
                    indexFirstGoodDelay.set(hs, delay);
                    tmpChipId.set(hs, id);
                }
                else if (indexSecondGoodDelay[hs] == -1)
                {
                    indexSecondGoodDelay.set(hs, delay);
                    tmpChipId.set(hs, id);
                }
            }
        }
    }

#if DEBUG_EMULATE_HEADSTAGES > 0
    if (tmpChipId[0] > 0)
    {
        int chipIdx = 0;
        for (int hs = 0; hs < DEBUG_EMULATE_HEADSTAGES && hs < headstages.size() ; ++hs)
        {
            if (enabledStreams.size() < MAX_NUM_DATA_STREAMS(evalBoard->isUSB3()))
            {
#ifdef DEBUG_EMULATE_64CH
                chipId.set(chipIdx++,CHIP_ID_RHD2164);
                chipId.set(chipIdx++,CHIP_ID_RHD2164_B);
                enableHeadstage(hs, true, 2, 32);
#else
                chipId.set(chipIdx++,CHIP_ID_RHD2132);
                enableHeadstage(hs, true, 1, 32);
#endif
            }
        }
        for (int i = 0; i < enabledStreams.size(); i++)
        {
            enabledStreams.set(i,Rhd2000EvalBoard::PortA1);
        }
    }

#else
    // Now, disable data streams where we did not find chips present.
    int chipIdx = 0;

    for (int hs = 0; hs < headstages.size(); ++hs)
    {
        if ((tmpChipId[hs] > 0) && (enabledStreams.size() < MAX_NUM_DATA_STREAMS))
        {
            chipId.set(chipIdx++,tmpChipId[hs]);

            LOGD("Enabling headstage ", hs);

            if (tmpChipId[hs] == CHIP_ID_RHD2164) //RHD2164
            {
                if (enabledStreams.size() < MAX_NUM_DATA_STREAMS - 1)
                {
                    enableHeadstage(hs, true, 2, 32);
                    chipId.set(chipIdx++, CHIP_ID_RHD2164_B);
                }
                else //just one stream left
                {
                    enableHeadstage(hs, true, 1, 32);
                }
            }
            else
            {
                enableHeadstage(hs, true, 1, tmpChipId[hs] == 1 ? 32:16);
            }
        }
        else
        {
            enableHeadstage(hs, false);
        }
    }
#endif
    updateBoardStreams();

    LOGD( "Number of enabled data streams: ", evalBoard->getNumEnabledDataStreams() );

    // Set cable delay settings that yield good communication with each
    // RHD2000 chip.
    Array<int> optimumDelay;

    optimumDelay.insertMultiple(0, 0, headstages.size());

    for (hs = 0; hs < headstages.size(); ++hs)
    {
        if (sumGoodDelays[hs] == 1 || sumGoodDelays[hs] == 2)
        {
            optimumDelay.set(hs,indexFirstGoodDelay[hs]);
        }
        else if (sumGoodDelays[hs] > 2)
        {
            optimumDelay.set(hs,indexSecondGoodDelay[hs]);
        }
    }

    evalBoard->setCableDelay(Rhd2000EvalBoard::PortA,
                             std::max(optimumDelay[0],optimumDelay[1]));
    evalBoard->setCableDelay(Rhd2000EvalBoard::PortB,
                             std::max(optimumDelay[2],optimumDelay[3]));
    evalBoard->setCableDelay(Rhd2000EvalBoard::PortC,
                             std::max(optimumDelay[4],optimumDelay[5]));
    evalBoard->setCableDelay(Rhd2000EvalBoard::PortD,
                             std::max(optimumDelay[6],optimumDelay[7]));

    if (boardType == RHD_RECORDING_CONTROLLER)
    {
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortE,
            std::max(optimumDelay[8], optimumDelay[9]));
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortF,
            std::max(optimumDelay[10], optimumDelay[11]));
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortG,
            std::max(optimumDelay[12], optimumDelay[13]));
        evalBoard->setCableDelay(Rhd2000EvalBoard::PortH,
            std::max(optimumDelay[14], optimumDelay[15]));

    }

    settings.cableLength.portA =
        evalBoard->estimateCableLengthMeters(std::max(optimumDelay[0],optimumDelay[1]));
    settings.cableLength.portB =
        evalBoard->estimateCableLengthMeters(std::max(optimumDelay[2],optimumDelay[3]));
    settings.cableLength.portC =
        evalBoard->estimateCableLengthMeters(std::max(optimumDelay[4],optimumDelay[5]));
    settings.cableLength.portD =
        evalBoard->estimateCableLengthMeters(std::max(optimumDelay[6],optimumDelay[7]));

    if (boardType == RHD_RECORDING_CONTROLLER)
    {
        settings.cableLength.portE =
            evalBoard->estimateCableLengthMeters(std::max(optimumDelay[8], optimumDelay[9]));
        settings.cableLength.portF =
            evalBoard->estimateCableLengthMeters(std::max(optimumDelay[10], optimumDelay[11]));
        settings.cableLength.portG =
            evalBoard->estimateCableLengthMeters(std::max(optimumDelay[12], optimumDelay[13]));
        settings.cableLength.portH =
            evalBoard->estimateCableLengthMeters(std::max(optimumDelay[14], optimumDelay[15]));
    }

    setSampleRate(settings.savedSampleRateIndex); // restore saved sample rate

    //updateRegisters();
    //newScan = true;
}



void DeviceThread::updateSettings(OwnedArray<ContinuousChannel>* continuousChannels,
    OwnedArray<EventChannel>* eventChannels,
    OwnedArray<SpikeChannel>* spikeChannels,
    OwnedArray<DataStream>* sourceStreams,
    OwnedArray<DeviceInfo>* devices,
    OwnedArray<ConfigurationObject>* configurationObjects)
{

    if (!deviceFound)
        return;

    continuousChannels->clear();
    eventChannels->clear();
    spikeChannels->clear();
    sourceStreams->clear();
    devices->clear();
    configurationObjects->clear();

    channelNames.clear();

    // create device
    // CODE GOES HERE

    DataStream::Settings dataStreamSettings
    {
        "Device Data",
        "description",
        "identifier",

        static_cast<float>(evalBoard->getSampleRate())

    };

    DataStream* stream = new DataStream(dataStreamSettings);

    sourceStreams->add(stream);

    int hsIndex = -1;

    for (auto headstage : headstages)
    {
        hsIndex++;

        if (headstage->isConnected())
        {
            for (int ch = 0; ch < headstage->getNumChannels(); ch++)
            {

                ContinuousChannel::Settings channelSettings{
                    ContinuousChannel::ELECTRODE,
                    headstage->getChannelName(ch),
                    "description",
                    "identifier",

                    0.195,

                    stream
                };

                continuousChannels->add(new ContinuousChannel(channelSettings));
                continuousChannels->getLast()->setUnits("uV");

                if (impedances.valid)
                {
                    continuousChannels->getLast()->impedance.magnitude = headstage->getImpedanceMagnitude(ch);
                    continuousChannels->getLast()->impedance.phase = headstage->getImpedancePhase(ch);
                }

            }

            if (settings.acquireAux)
            {
                for (int ch = 0; ch < 3; ch++)
                {

                    ContinuousChannel::Settings channelSettings{
                        ContinuousChannel::AUX,
                        headstage->getStreamPrefix() + "_AUX" + String(ch + 1),
                        "description",
                        "identifier",

                        0.0000374,

                        stream
                    };

                    continuousChannels->add(new ContinuousChannel(channelSettings));
                    continuousChannels->getLast()->setUnits("mV");

                }
            }
        }
    }

    if (settings.acquireAdc)
    {
        for (int ch = 0; ch < 8; ch++)
        {

            String name = "ADC" + String(ch + 1);

            ContinuousChannel::Settings channelSettings{
                ContinuousChannel::ADC,
                name,
                "description",
                "identifier",

                getAdcBitVolts(ch),

                stream
            };

            continuousChannels->add(new ContinuousChannel(channelSettings));
            continuousChannels->getLast()->setUnits("V");

        }
    }

    EventChannel::Settings settings{
            EventChannel::Type::TTL,
            "name",
            "description",
            "identifier",
            stream,
            8
    };

    eventChannels->add(new EventChannel(settings));

}

void DeviceThread::impedanceMeasurementFinished()
{

    if (impedances.valid)
    {
        LOGD( "Updating headstage impedance values" );

        for (auto hs : headstages)
        {
            if (hs->isConnected())
            {
                hs->setImpedances(impedances);
            }
        }
    }
}

void DeviceThread::saveImpedances(File& file)
{

    if (impedances.valid)
    {
        std::unique_ptr<XmlElement> xml = std::unique_ptr<XmlElement>(new XmlElement("IMPEDANCES"));

        int globalChannelNumber = -1;

        for (auto hs : headstages)
        {
            XmlElement* headstageXml = new XmlElement("HEADSTAGE");
            headstageXml->setAttribute("name", hs->getStreamPrefix());

            for (int ch = 0; ch < hs->getNumActiveChannels(); ch++)
            {
                globalChannelNumber++;

                XmlElement* channelXml = new XmlElement("CHANNEL");
                channelXml->setAttribute("name", hs->getChannelName(ch));
                channelXml->setAttribute("number", globalChannelNumber);
                channelXml->setAttribute("magnitude", hs->getImpedanceMagnitude(ch));
                channelXml->setAttribute("phase", hs->getImpedancePhase(ch));
                headstageXml->addChildElement(channelXml);
            }

            xml->addChildElement(headstageXml);
        }

        xml->writeTo(file);
    }

}

String DeviceThread::getChannelName(int i) const
{
    return channelNames[i];
}

bool DeviceThread::isAcquisitionActive() const
{
    return isTransmitting;
}

void DeviceThread::setNamingScheme(ChannelNamingScheme scheme)
{

    channelNamingScheme = scheme;

    for (auto hs : headstages)
    {
        hs->setNamingScheme(scheme);
    }
}

ChannelNamingScheme DeviceThread::getNamingScheme()
{
    return channelNamingScheme;
}

void DeviceThread::setNumChannels(int hsNum, int numChannels)
{
    if (headstages[hsNum]->getNumChannels() == 32)
    {
        if (numChannels < headstages[hsNum]->getNumChannels())
            headstages[hsNum]->setHalfChannels(true);
        else
            headstages[hsNum]->setHalfChannels(false);

        numChannelsPerDataStream.set(headstages[hsNum]->getStreamIndex(0), numChannels);
    }

    int channelIndex = 0;

    for (auto hs : headstages)
    {
        if (hs->isConnected())
        {
            hs->setFirstChannel(channelIndex);

            channelIndex += hs->getNumActiveChannels();
        }
    }
}

int DeviceThread::getHeadstageChannels (int hsNum) const
{
    return headstages[hsNum]->getNumChannels();
}


int DeviceThread::getNumChannels()
{
    int totalChannels = getNumDataOutputs(ContinuousChannel::ELECTRODE)
           + getNumDataOutputs(ContinuousChannel::AUX)
           + getNumDataOutputs(ContinuousChannel::ADC);

    return totalChannels;
}

int DeviceThread::getNumDataOutputs(ContinuousChannel::Type type)
{

    if (type == ContinuousChannel::ELECTRODE)
    {
        int totalChannels = 0;

        for (auto headstage : headstages)
        {
            if (headstage->isConnected())
            {
                totalChannels += headstage->getNumActiveChannels();
            }
        }

        return totalChannels;
    }
    if (type == ContinuousChannel::AUX)
    {
        if (settings.acquireAux)
        {
            int numAuxOutputs = 0;

            for (auto headstage : headstages)
            {
                if (headstage->isConnected())
                {
                    numAuxOutputs += 3;
                }
            }
            return numAuxOutputs;
        }
        else
        {
            return 0;
        }
    }
    if (type == ContinuousChannel::ADC)
    {
        if (settings.acquireAdc)
        {
            return 8;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}


float DeviceThread::getAdcBitVolts (int chan) const
{
    if (chan < adcBitVolts.size())
    {
        return adcBitVolts[chan];
    }
    else
    {
        if (boardType == ACQUISITION_BOARD)
        {
            return 0.00015258789; // +/-5V / pow(2,16)
        }
        else if (boardType == INTAN_RHD_USB)
        {
            return 0.0000503540039;  // 3.3V / pow(2,16)
        }
    }
}

double DeviceThread::setUpperBandwidth(double upper)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.upperBandwidth = upper;

    updateRegisters();

    return settings.dsp.upperBandwidth;
}


double DeviceThread::setLowerBandwidth(double lower)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.lowerBandwidth = lower;

    updateRegisters();

    return settings.dsp.lowerBandwidth;
}

double DeviceThread::setDspCutoffFreq(double freq)
{
    impedanceThread->stopThreadSafely();

    settings.dsp.cutoffFreq = freq;

    updateRegisters();

    return settings.dsp.cutoffFreq;
}

double DeviceThread::getDspCutoffFreq() const
{
    return settings.dsp.cutoffFreq;
}

void DeviceThread::setDSPOffset(bool state)
{

    impedanceThread->stopThreadSafely();

    settings.dsp.enabled = state;

    updateRegisters();
}

void DeviceThread::setTTLoutputMode(bool state)
{
    settings.ttlMode = state;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setDAChpf(float cutoff, bool enabled)
{
    settings.desiredDAChpf = cutoff;

    settings.desiredDAChpfState = enabled;

    updateSettingsDuringAcquisition = true;
}

void DeviceThread::setFastTTLSettle(bool state, int channel)
{
    settings.fastTTLSettleEnabled = state;

    settings.fastSettleTTLChannel = channel;

    updateSettingsDuringAcquisition = true;
}

int DeviceThread::setNoiseSlicerLevel(int level)
{
    settings.noiseSlicerLevel = level;

    if (deviceFound)
        evalBoard->setAudioNoiseSuppress(settings.noiseSlicerLevel);

    // Level has been checked once before this and then is checked again in setAudioNoiseSuppress.
    // This may be overkill - maybe API should change so that the final function returns the value?

    return settings.noiseSlicerLevel;
}


bool DeviceThread::foundInputSource()
{
    return deviceFound;
}

bool DeviceThread::enableHeadstage(int hsNum, bool enabled, int nStr, int strChans)
{

    if (enabled)
    {
        headstages[hsNum]->setFirstChannel(getNumDataOutputs(ContinuousChannel::ELECTRODE));
        headstages[hsNum]->setNumStreams(nStr);
        headstages[hsNum]->setChannelsPerStream(strChans);
        headstages[hsNum]->setFirstStreamIndex(enabledStreams.size());
        enabledStreams.add(headstages[hsNum]->getDataStream(0));
        numChannelsPerDataStream.add(strChans);

        if (nStr > 1)
        {
            enabledStreams.add(headstages[hsNum]->getDataStream(1));
            numChannelsPerDataStream.add(strChans);
        }
    }
    else
    {
        int idx = enabledStreams.indexOf(headstages[hsNum]->getDataStream(0));

        if (idx >= 0)
        {
            enabledStreams.remove(idx);
            numChannelsPerDataStream.remove(idx);
        }

        if (headstages[hsNum]->getNumStreams() > 1)
        {
            idx = enabledStreams.indexOf(headstages[hsNum]->getDataStream(1));
            if (idx >= 0)
            {
                enabledStreams.remove(idx);
                numChannelsPerDataStream.remove(idx);
            }
        }

        headstages[hsNum]->setNumStreams(0);
    }

    sourceBuffers[0]->resize(getNumChannels(), 10000);

    return true;
}

void DeviceThread::updateBoardStreams()
{
    for (int i = 0; i < MAX_NUM_DATA_STREAMS; i++)
    {
        if (i < enabledStreams.size())
        {
            evalBoard->enableDataStream(i,true);
            evalBoard->setDataSource(i,enabledStreams[i]);
        }
        else
        {
            evalBoard->enableDataStream(i,false);
        }
    }
}

bool DeviceThread::isHeadstageEnabled(int hsNum) const
{
    return headstages[hsNum]->isConnected();
}


int DeviceThread::getActiveChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumActiveChannels();
}

int DeviceThread::getChannelsInHeadstage (int hsNum) const
{
    return headstages[hsNum]->getNumChannels();
}

/*void DeviceThread::assignAudioOut(int dacChannel, int dataChannel)
{
    if (deviceFound)
    {
        if (dacChannel == 0)
        {
            audioOutputR = dataChannel;
            dacChannels[0] = dataChannel;
        }
        else if (dacChannel == 1)
        {
            audioOutputL = dataChannel;
            dacChannels[1] = dataChannel;
        }

        updateSettingsDuringAcquisition = true; // set a flag and take care of setting wires
        // during the updateBuffer() method
        // to avoid problems
    }

}*/

void DeviceThread::enableAuxs(bool t)
{
    settings.acquireAux = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
    updateRegisters();
}

void DeviceThread::enableAdcs(bool t)
{
    settings.acquireAdc = t;
    sourceBuffers[0]->resize(getNumChannels(), 10000);
}

bool DeviceThread::isAuxEnabled()
{
    return settings.acquireAux;
}

void DeviceThread::setSampleRate(int sampleRateIndex, bool isTemporary)
{
    impedanceThread->stopThreadSafely();
    
    if (!isTemporary)
    {
        device->settings.savedSampleRateIndex = sampleRateIndex;
    }
    
    device->setParameter(SAMPLE_RATE, sampleRateIndex);
}

bool DeviceThread::startAcquisition()
{
    
    impedanceThread->waitSafely();
    
    ErrorCode errorCode = device->startAcquisition();
    
    // reset TTL output state
    for (int k = 0; k < 16; k++)
    {
        TTL_OUTPUT_STATE[k] = 0;
    }
    
    startThread();

    return true;
}

bool DeviceThread::stopAcquisition()
{

    LOGD("DeviceThread stopping acquisition.");

    if (isThreadRunning())
    {
        signalThreadShouldExit();

    }

    if (waitForThreadToExit(500))
    {
        LOGD("DeviceThread exited.");
    }
    else
    {
        LOGD("DeviceThread failed to exit, continuing anyway...");
    }

    sourceBuffers[0]->clear();
    
    isTransmitting = false;
    updateSettingsDuringAcquisition = false;

    // remove timers
    digitalOutputTimers.clear();

    // remove commands
    while (!digitalOutputCommands.empty())
        digitalOutputCommands.pop();

    return true;
}

bool DeviceThread::updateBuffer()
{
            
    ErrorCode erroCode = device->fillDataBlock(&dataBlock);

    sourceBuffers[0]->addToBuffer(dataBlock.samples.data(),
                                  dataBlock.sampleNumbers.data(),
                                  dataBlock.timestamps.data(),
                                  dataBlock.ttlEventWords.data(),
                                  nSamps);

    if (updateSettingsDuringAcquisition)
    {
        for (int k = 0; k < 8; k++)
        {
            if (dacChannelsToUpdate[k])
            {
                dacChannelsToUpdate[k] = false;
                
                if (dacChannels[k] >= 0)
                {
                    evalBoard->enableDac(k, true);
                    evalBoard->selectDacDataStream(k, dacStream[k]);
                    evalBoard->selectDacDataChannel(k, dacChannels[k]);
                    evalBoard->setDacThreshold(k, (int)abs((dacThresholds[k]/0.195) + 32768),dacThresholds[k] >= 0);
                   // evalBoard->setDacThresholdVoltage(k, (int) dacThresholds[k]);
                }
                else
                {
                    evalBoard->enableDac(k, false);
                }
            }
        }

        evalBoard->setTtlMode(settings.ttlMode ? 1 : 0);
        evalBoard->enableExternalFastSettle(settings.fastTTLSettleEnabled);
        evalBoard->setExternalFastSettleChannel(settings.fastSettleTTLChannel);
        evalBoard->setDacHighpassFilter(settings.desiredDAChpf);
        evalBoard->enableDacHighpassFilter(settings.desiredDAChpfState);
        evalBoard->enableBoardLeds(settings.ledsEnabled);
        evalBoard->setClockDivider(settings.clockDivideFactor);

        updateSettingsDuringAcquisition = false;
    }

    if (!digitalOutputCommands.empty())
    {

        while (!digitalOutputCommands.empty())
        {
            DigitalOutputCommand command = digitalOutputCommands.front();
            TTL_OUTPUT_STATE[command.ttlLine] = command.state;
            digitalOutputCommands.pop();

        }

        device->setDigitalOut(TTL_OUTPUT_STATE);

        LOGB("TTL OUTPUT STATE: ",
            TTL_OUTPUT_STATE[0],
            TTL_OUTPUT_STATE[1],
            TTL_OUTPUT_STATE[2],
            TTL_OUTPUT_STATE[3],
            TTL_OUTPUT_STATE[4],
            TTL_OUTPUT_STATE[5],
            TTL_OUTPUT_STATE[6],
            TTL_OUTPUT_STATE[7]);

    }

    return true;

}

int DeviceThread::getChannelFromHeadstage (int hs, int ch)
{
    int channelCount = 0;
    int hsCount = 0;
    if (hs < 0 || hs >= headstages.size() + 1)
        return -1;
    if (hs == headstages.size()) //let's consider this the ADC channels
    {
        int adcOutputs = getNumDataOutputs(ContinuousChannel::ADC);

        if (adcOutputs> 0)
        {
            return getNumDataOutputs(ContinuousChannel::ELECTRODE) + getNumDataOutputs(ContinuousChannel::AUX) + ch;
        }
        else
            return -1;
    }
    if (headstages[hs]->isConnected())
    {
        if (ch < 0)
            return -1;
        if (ch < headstages[hs]->getNumActiveChannels())
        {
            for (int i = 0; i < hs; i++)
            {
                channelCount += headstages[i]->getNumActiveChannels();
            }
            return channelCount + ch;
        }
        else if (ch < headstages[hs]->getNumActiveChannels() + 3)
        {
            for (int i = 0; i < headstages.size(); i++)
            {
                if (headstages[i]->isConnected())
                {
                    channelCount += headstages[i]->getNumActiveChannels();
                    if (i < hs)
                        hsCount++;
                }
            }
            return channelCount + hsCount * 3 + ch-headstages[hs]->getNumActiveChannels();
        }
        else
        {
            return -1;
        }

    }
    else
    {
        return -1;
    }
}

Array<const Headstage*> DeviceThread::getConnectedHeadstages()
{
    Array<const Headstage*> headstageArray;

    for (auto hs : headstages)
    {
        if (hs->isConnected())
            headstageArray.add(hs);
    }

    return headstageArray;
}

int DeviceThread::getHeadstageChannel (int& hs, int ch) const
{
    int channelCount = 0;
    int hsCount = 0;

    if (ch < 0)
        return -1;

    for (int i = 0; i < headstages.size(); i++)
    {
        if (headstages[i]->isConnected())
        {
            int chans = headstages[i]->getNumActiveChannels();

            if (ch >= channelCount && ch < channelCount + chans)
            {
                hs = i;
                return ch - channelCount;
            }
            channelCount += chans;
            hsCount++;
        }
    }
    if (ch < (channelCount + hsCount * 3)) //AUX
    {
        hsCount = (ch - channelCount) / 3;

        for (int i = 0; i < headstages.size(); i++)
        {
            if (headstages[i]->isConnected())
            {
                if (hsCount == 0)
                {
                    hs = i;
                    return ch - channelCount;
                }
                hsCount--;
                channelCount++;
            }
        }
    }
    return -1;
}

void DeviceThread::enableBoardLeds(bool enable)
{
    device->settings.ledsEnabled = enable;

    if (isAcquisitionActive())
        updateSettingsDuringAcquisition = true;
    else
        device->setParameter(ENABLE_LEDS, enable);
}

int DeviceThread::setClockDivider(int divide_ratio)
{
    if (!deviceFound)
        return 1;

    // Divide ratio should be 1 or an even number
    if (divide_ratio != 1 && divide_ratio % 2)
        divide_ratio--;

    // Format the divide ratio from its true value to the
    // format required by the firmware
    // Ratio    N
    // 1        0
    // >=2      Ratio/2
    if (divide_ratio == 1)
        device->settings.clockDivideFactor = 0;
    else
        device->settings.clockDivideFactor = static_cast<uint16>(divide_ratio/2);

    if (isAcquisitionActive())
        updateSettingsDuringAcquisition = true;
    else
        device->setParameter(CLOCK_DIVIDE_FACTOR, device->settings.clockDivideFactor);

    return divide_ratio;
}

void DeviceThread::setAdcRange(int channel, short range)
{
    adcRangeSettings[channel] = range;
}

short DeviceThread::getAdcRange(int channel) const
{
    return adcRangeSettings[channel];
}

void DeviceThread::runImpedanceTest()
{

    impedanceThread->stopThreadSafely();

    impedanceThread->runThread();

}



